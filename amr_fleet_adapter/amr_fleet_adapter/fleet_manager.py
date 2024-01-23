#!/usr/bin/env python3

# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import math
import yaml
import json
import time
import copy
import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability

from rmf_fleet_msgs.msg import FleetState, RobotState, Location, PathRequest, \
    DockRequest, ModeRequest, CancelRequest, DockSummary, RobotMode, DockMode

import rmf_adapter as adpt
import rmf_adapter.vehicletraits as traits
import rmf_adapter.geometry as geometry

import numpy as np
from pyproj import Transformer

from fastapi import FastAPI
import uvicorn
from typing import Optional
from pydantic import BaseModel

import threading
app = FastAPI()


class Request(BaseModel):
    map_name: Optional[str] = None
    task: Optional[dict] = None
    destination: Optional[dict] = None
    waypoints: Optional[list] = None
    data: Optional[dict] = None
    speed_limit: Optional[float] = None
    toggle: Optional[bool] = None


class Response(BaseModel):
    data: Optional[dict] = None
    success: bool
    msg: str


# ------------------------------------------------------------------------------
# Fleet Manager
# ------------------------------------------------------------------------------
class State:
    def __init__(self, state: RobotState = None, destination: Location = None, path: list = None):
        self.state = state
        self.destination = destination
        self.path = path
        self.last_request = None
        self.last_completed_request = None
        self.mode_teleop = False
        self.svy_transformer = Transformer.from_crs('EPSG:4326', 'EPSG:3414')
        self.gps_pos = [0, 0]

    def gps_to_xy(self, gps_json: dict):
        svy21_xy = \
            self.svy_transformer.transform(gps_json['lat'], gps_json['lon'])
        self.gps_pos[0] = svy21_xy[1]
        self.gps_pos[1] = svy21_xy[0]

    def is_expected_task_id(self, task_id):
        if self.last_request is not None:
            if task_id != self.last_request.task_id:
                return False
        return True


class FleetManager(Node):
    def __init__(self, config, nav_path):
        self.debug = False
        self.config = config
        self.fleet_name = self.config["rmf_fleet"]["name"]

        self.gps = False
        self.offset = [0, 0]
        if 'reference_coordinates' in self.config and \
                'offset' in self.config['reference_coordinates']:
            assert len(self.config['reference_coordinates']['offset']) > 1, \
                ('Please ensure that the offset provided is valid.')
            self.gps = True
            self.offset = self.config['reference_coordinates']['offset']

        super().__init__(f'{self.fleet_name}_fleet_manager')

        self.robots = {}  # Map robot name to state
        self.docks = {} # Map dock name to waypoints

        if 'docks' in self.config:
            self.docks = self.config['docks']
        assert(len(self.docks) > 0)

        for robot_name, robot_config in self.config["robots"].items():
            self.robots[robot_name] = State()
        assert(len(self.robots) > 0)

        profile = traits.Profile(geometry.make_final_convex_circle(
            self.config['rmf_fleet']['profile']['footprint']),
            geometry.make_final_convex_circle(
                self.config['rmf_fleet']['profile']['vicinity']))
        self.vehicle_traits = traits.VehicleTraits(
            linear=traits.Limits(
                *self.config['rmf_fleet']['limits']['linear']),
            angular=traits.Limits(
                *self.config['rmf_fleet']['limits']['angular']),
            profile=profile)
        self.vehicle_traits.differential.reversible =\
            self.config['rmf_fleet']['reversible']

        self.create_subscription(
            FleetState,
            'fleet_states',
            self.fleet_states_cb,
            100
        )

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,   
            durability=Durability.TRANSIENT_LOCAL)

        self.create_subscription(
            DockSummary,
            'dock_summary',
            self.dock_summary_cb,
            qos_profile=transient_qos)

        self.path_pub = self.create_publisher(
            PathRequest,
            'robot_path_requests',
            qos_profile=qos_profile_system_default)

        self.dock_pub = self.create_publisher(
            DockRequest,
            'robot_dock_requests',
            qos_profile=qos_profile_system_default)

        self.mode_pub = self.create_publisher(
            ModeRequest,
            'robot_mode_requests',
            qos_profile=qos_profile_system_default)
        
        self.cancel_pub = self.create_publisher(
            CancelRequest,
            'robot_cancel_requests',
            qos_profile=qos_profile_system_default)


        @app.get('/vdm-rmf/data/status/',
                 response_model=Response)
        async def status(robot_name: Optional[str] = None):
            response = {
                'data': {},
                'success': False,
                'msg': ''
            }
            if robot_name is None:
                response['data']['all_robots'] = []
                for robot_name in self.robots:
                    state = self.robots.get(robot_name)
                    if state is None or state.state is None:
                        return response
                    response['data']['all_robots'].append(
                        self.get_robot_state(state, robot_name))
            else:
                state = self.robots.get(robot_name)
                if state is None or state.state is None:
                    return response
                response['data'] = self.get_robot_state(state, robot_name)
            response['success'] = True
            return response


        @app.post('/vdm-rmf/cmd/navigate/',
                  response_model=Response)
        async def navigate(robot_name: str, cmd_id: int, dest: Request):
            response = {'success': False, 'msg': ''}
            if (robot_name not in self.robots or len(dest.destination) < 1):
                return response

            robot = self.robots[robot_name]

            target_x = dest.destination['x']
            target_y = dest.destination['y']
            target_yaw = dest.destination['yaw']
            target_map = dest.map_name
            target_speed_limit = dest.speed_limit

            target_x -= self.offset[0]
            target_y -= self.offset[1]

            t = self.get_clock().now().to_msg()

            path_request = PathRequest()
            robot = self.robots[robot_name]
            cur_x = robot.state.location.x
            cur_y = robot.state.location.y
            cur_yaw = robot.state.location.yaw
            cur_loc = robot.state.location
            path_request.path.append(cur_loc)

            disp = self.disp([target_x, target_y], [cur_x, cur_y])
            duration = int(disp/self.vehicle_traits.linear.nominal_velocity) +\
                int(abs(abs(cur_yaw) - abs(target_yaw)) /
                    self.vehicle_traits.rotational.nominal_velocity)
            t.sec = t.sec + duration
            target_loc = Location()
            target_loc.t = t
            target_loc.x = target_x
            target_loc.y = target_y
            target_loc.yaw = target_yaw
            target_loc.level_name = target_map
            if target_speed_limit > 0:
                target_loc.obey_approach_speed_limit = True
                target_loc.approach_speed_limit = target_speed_limit

            path_request.fleet_name = self.fleet_name
            path_request.robot_name = robot_name
            path_request.path.append(target_loc)
            path_request.task_id = str(cmd_id)
            self.path_pub.publish(path_request)

            if self.debug:
                print(f'Sending navigate request for {robot_name}: {cmd_id}')
            robot.last_request = path_request
            robot.destination = target_loc

            response['success'] = True
            return response


        @app.post('/vdm-rmf/cmd/follow_path/',
                  response_model=Response)
        async def follow_path(robot_name: str, cmd_id: int, dest: Request):
            response = {'success': False, 'msg': ''}
            if (robot_name not in self.robots or len(dest.waypoints) < 1):
                return response

            robot = self.robots[robot_name]
            
            duration = 0
            path_request = PathRequest()
            cur_x = robot.state.location.x
            cur_y = robot.state.location.y
            cur_yaw = robot.state.location.yaw
            cur_loc = robot.state.location
            path_request.path.append(cur_loc)
            t = self.get_clock().now().to_msg()
            for destination in dest.waypoints:
                target_x = destination['x']
                target_y = destination['y']
                target_yaw = destination['yaw']
                target_speed_limit = destination['speed_limit']
                target_map = dest.map_name

                target_x -= self.offset[0]
                target_y -= self.offset[1]
            
                disp = self.disp([target_x, target_y], [cur_x, cur_y])
                duration = int(disp/self.vehicle_traits.linear.nominal_velocity) +\
                    int(abs(abs(cur_yaw) - abs(target_yaw)) /
                        self.vehicle_traits.rotational.nominal_velocity)

                t.sec = t.sec + duration
                target_loc = Location()
                target_loc.t = t
                target_loc.x = target_x
                target_loc.y = target_y
                target_loc.yaw = target_yaw
                target_loc.level_name = target_map
                target_loc.obey_approach_speed_limit = False
                if target_speed_limit is not None and target_speed_limit > 0.0:
                    target_loc.obey_approach_speed_limit = True
                    target_loc.approach_speed_limit = target_speed_limit

                path_request.path.append(target_loc)
                cur_x = target_x
                cur_y = target_y
                cur_yaw = target_yaw

            path_request.task_id = str(cmd_id)
            path_request.fleet_name = self.fleet_name
            path_request.robot_name = robot_name
            self.path_pub.publish(path_request)

            if self.debug:
                print(f'Sending follow_path request for {robot_name}: {cmd_id}')
            robot.last_request = path_request
            robot.destination = target_loc
            robot.path = path_request.path[1:]
            
            response['success'] = True
            return response


        @app.get('/vdm-rmf/cmd/pause_robot/',
                 response_model=Response)
        async def pause(robot_name: str, cmd_id: int):
            response = {'success': False, 'msg': ''}
            if robot_name not in self.robots:
                return response

            robot = self.robots[robot_name]
            mode_request = ModeRequest()
            mode_request.fleet_name = self.fleet_name
            mode_request.robot_name = robot_name
            mode_request.mode.mode = RobotMode.MODE_PAUSED

            mode_request.task_id = str(cmd_id)
            self.mode_pub.publish(mode_request)

            if self.debug:
                print(f'Sending pause request for {robot_name}: {cmd_id}')
            robot.last_request = mode_request

            response['success'] = True
            return response


        @app.get('/vdm-rmf/cmd/resume_robot/',
                 response_model=Response)
        async def resume(robot_name: str, cmd_id: int):
            response = {'success': False, 'msg': ''}
            if robot_name not in self.robots:
                return response

            robot = self.robots[robot_name]
            mode_request = ModeRequest()
            mode_request.fleet_name = self.fleet_name
            mode_request.robot_name = robot_name
            mode_request.mode.mode = RobotMode.MODE_MOVING

            mode_request.task_id = str(cmd_id)
            self.mode_pub.publish(mode_request)

            if self.debug:
                print(f'Sending resume request for {robot_name}: {cmd_id}')
            robot.last_request = mode_request

            response['success'] = True
            return response


        @app.get('/vdm-rmf/cmd/stop_robot/',
                 response_model=Response)
        async def stop(robot_name: str, cmd_id: int):
            response = {'success': False, 'msg': ''}
            if robot_name not in self.robots:
                return response

            robot = self.robots[robot_name]
            cancel_request = CancelRequest()
            cancel_request.fleet_name = self.fleet_name
            cancel_request.robot_name = robot_name
            cancel_request.task_id = str(cmd_id)
            self.cancel_pub.publish(cancel_request)

            if self.debug:
                print(f'Sending stop request for {robot_name}: {cmd_id}')
            robot.last_request = cancel_request
            robot.destination = None
            robot.path = None

            response['success'] = True
            return response


        @app.post('/vdm-rmf/cmd/start_task/',
                  response_model=Response)
        async def start_process(robot_name: str, cmd_id: int, task: Request):
            response = {'success': False, 'msg': ''}
            if (robot_name not in self.robots or
                    len(task.task) < 2):
                return response

            robot = self.robots[robot_name]
            custom_dock = False
            rotate_to_dock = 0
            rotate_angle = 0
            rotate_orientation = 0



            dock_request = DockRequest()
            if task.task["mode"] == "charge":
                dock_request.dock_mode.mode = DockMode.MODE_CHARGE
            elif task.task["mode"] == "pickup":
                dock_request.dock_mode.mode = DockMode.MODE_PICKUP
            elif task.task["mode"] == "dropoff":
                dock_request.dock_mode.mode = DockMode.MODE_DROPOFF
            elif task.task["mode"] == "undock":
                dock_request.dock_mode.mode = DockMode.MODE_UNDOCK
            else:
                response["msg"] = "Mode dock does not support. Please check mode!"
                return response

            if task.task["dock_name"] in self.docks:
                dock_config = self.docks[task.task["dock_name"]]
                for conf in dock_config:
                    if conf == 'custom_dock':
                        custom_dock = dock_config[conf]
                    elif conf == 'rotate_to_dock':
                        rotate_to_dock = dock_config[conf]
                    elif conf == 'rotate_angle':
                        rotate_angle = dock_config[conf]
                    elif conf == 'rotate_orientation':
                        rotate_orientation = dock_config[conf]
                    else:
                        continue
                
            else:
                response["msg"] = "Not found dock name in config!"
                return response

            destination = Location()
            destination.level_name = task.map_name
            destination.x = task.task["location"][0]
            destination.y = task.task["location"][1]
            destination.yaw = task.task["location"][2]
            dock_request.destination = destination
            dock_request.fleet_name = self.fleet_name
            dock_request.robot_name = robot_name
            dock_request.custom_docking = custom_dock
            dock_request.rotate_to_dock = rotate_to_dock
            dock_request.rotate_angle = rotate_angle
            dock_request.rotate_orientation = rotate_orientation
            dock_request.task_id = str(cmd_id)

            self.dock_pub.publish(dock_request)

            if self.debug:
                print(f'Sending process request for {robot_name}: {cmd_id}')
            robot.last_request = dock_request
            robot.destination = destination

            response['success'] = True
            return response


        @app.post('/vdm-rmf/cmd/toggle_action/',
                  response_model=Response)
        async def toggle_teleop(robot_name: str, mode: Request):
            response = {'success': False, 'msg': ''}
            if (robot_name not in self.robots):
                return response
            # Toggle action mode
            self.robots[robot_name].mode_teleop = mode.toggle
            response['success'] = True
            return response
        

        # @app.get('/vdm-rmf/cmd/is_task_queue_finished/',
        #          response_model=Response)
        # async def is_task_queue_finished(robot_name: str):
        #     response = {
        #         'data': {},
        #         'success': False,
        #         'msg': ''
        #     }
        #     if (robot_name not in self.robots):
        #         return response

        #     robot = self.robots[robot_name]
        #     if robot.last_completed_request == robot.last_request.task_id:
        #         response['data']['task_finished'] = True
        #     else:
        #         response['data']['task_finished'] = False
        #     response['success'] = True
        #     return response


    def fleet_states_cb(self, msg: FleetState):
        if (msg.name == self.fleet_name):
            for robotMsg in msg.robots:
                if (robotMsg.name in self.robots):
                    robot = self.robots[robotMsg.name]
                    if not robot.is_expected_task_id(robotMsg.task_id) and \
                            not robot.mode_teleop:
                        # This message is out of date, so disregard it.
                        if robot.last_request is not None:
                            # Resend the latest task request for this robot, in case
                            # the message was dropped.
                            if self.debug:
                                print(
                                    f'Republishing task request for {robotMsg.name}: '
                                    f'{robot.last_request.task_id}, '
                                    f'because it is currently following {robotMsg.task_id}'
                                )
                            if type(robot.last_request) is PathRequest:
                                self.path_pub.publish(robot.last_request)
                            elif type(robot.last_request) is ModeRequest:
                                self.mode_pub.publish(robot.last_request)
                            elif type(robot.last_request) is DockRequest:
                                self.dock_pub.publish(robot.last_request)
                            elif type(robot.last_request is CancelRequest):
                                self.cancel_pub.publish(robot.last_request)
                        return

                    robot.state = robotMsg
                    # Check if robot has reached destination
                    if robot.destination is None:
                        return

                    if (
                        (
                            robotMsg.mode.mode == RobotMode.MODE_IDLE
                            or robotMsg.mode.mode == RobotMode.MODE_CHARGING
                        )
                        and len(robotMsg.path) == 0
                    ):
                        robot = self.robots[robotMsg.name]
                        robot.destination = None
                        robot.path = None
                        completed_request = int(robotMsg.task_id)
                        if robot.last_completed_request != completed_request:
                            if self.debug:
                                print(
                                    f'Detecting completed request for {robotMsg.name}: '
                                    f'{completed_request}'
                                )
                        robot.last_completed_request = completed_request

    def dock_summary_cb(self, msg):
        return
        for fleet in msg.docks:
            if(fleet.fleet_name == self.fleet_name):
                for dock in fleet.params:
                    self.docks[dock.start] = dock.path

    def get_robot_state(self, robot: State, robot_name):
        data = {}
        if self.gps:
            position = copy.deepcopy(robot.gps_pos)
        else:
            position = [robot.state.location.x, robot.state.location.y]
        angle = robot.state.location.yaw
        data['robot_name'] = robot_name
        data['map_name'] = robot.state.location.level_name
        data['position'] =\
            {'x': position[0], 'y': position[1], 'yaw': angle}
        data['battery'] = robot.state.battery_percent
        if (robot.destination is not None
            and robot.last_request is not None
            and robot.path is not None):
            data['path_length'] = len(robot.path)
            # Sửa destination để nhận điểm đầu tiên trong path
            # destination = robot.destination
            destination = robot.path[0]
            n = len(robot.path) - len(robot.state.path)
            if (len(robot.state.path) > 0
                and n > 0):
                robot.path = robot.path[n:]

            # remove offset for calculation if using gps coords
            if self.gps:
                position[0] -= self.offset[0]
                position[1] -= self.offset[1]
            # calculate arrival estimate
            dist_to_target =\
                self.disp(position, [destination.x, destination.y])
            ori_delta = abs(abs(angle) - abs(destination.yaw))
            if ori_delta > np.pi:
                ori_delta = ori_delta - (2 * np.pi)
            if ori_delta < -np.pi:
                ori_delta = (2 * np.pi) + ori_delta
            duration = (dist_to_target /
                        self.vehicle_traits.linear.nominal_velocity +
                        ori_delta /
                        self.vehicle_traits.rotational.nominal_velocity)
            cmd_id = int(robot.last_request.task_id)
            data['destination_arrival'] = {
                'cmd_id': cmd_id,
                'duration': duration
            }
        else:
            data['destination_arrival'] = None
            data['path_length'] = None

        data['last_completed_request'] = robot.last_completed_request
        if (
            robot.state.mode.mode == RobotMode.MODE_WAITING
            or robot.state.mode.mode == RobotMode.MODE_ADAPTER_ERROR
        ):
            # The name of MODE_WAITING is not very intuitive, but the slotcar
            # plugin uses it to indicate when another robot is blocking its
            # path.
            #
            # MODE_ADAPTER_ERROR means the robot received a plan that
            # didn't make sense, i.e. the plan expected the robot was starting
            # very far from its real present location. When that happens we
            # should replan, so we'll set replan to true in that case as well.
            data['replan'] = True
        else:
            data['replan'] = False

        data['robot_mode'] = robot.state.mode.mode

        return data

    def disp(self, A, B):
        return math.sqrt((A[0]-B[0])**2 + (A[1]-B[1])**2)


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def main(argv=sys.argv):
    # Init rclpy and adapter
    rclpy.init(args=argv)
    adpt.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="fleet_adapter",
        description="Configure and spin up the fleet adapter")
    parser.add_argument("-c", "--config_file", type=str, required=True,
                        help="Path to the config.yaml file")
    parser.add_argument("-n", "--nav_graph", type=str, required=True,
                        help="Path to the nav_graph for this fleet adapter")
    args = parser.parse_args(args_without_ros[1:])
    print(f"Starting fleet manager...")

    with open(args.config_file, "r") as f:
        config = yaml.safe_load(f)

    fleet_manager = FleetManager(config, args.nav_graph)

    spin_thread = threading.Thread(target=rclpy.spin, args=(fleet_manager,))
    spin_thread.start()

    uvicorn.run(
        app,
        host=config['rmf_fleet']['fleet_manager']['ip'],
        port=config['rmf_fleet']['fleet_manager']['port'],
        log_level='warning'
    )


if __name__ == '__main__':
    main(sys.argv)
