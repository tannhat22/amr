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

import argparse
import copy
import json
import math
import sys
import threading
import time
import re
from typing import Optional

from fastapi import FastAPI
import numpy as np
from pydantic import BaseModel
from pyproj import Transformer
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy as Reliability
import rmf_adapter as adpt
import rmf_adapter.geometry as geometry
import rmf_adapter.vehicletraits as traits
from charger_fleet_msgs.msg import ChargerRequest, ChargerMode
from rmf_fleet_msgs.msg import (
    FleetState,
    RobotState,
    RobotMode,
    DockLimit,
    DockMode,
    DockParam,
    DockTag,
    DockSummary,
    Location,
    CancelRequest,
    DockRequest,
    LocalizeRequest,
    ModeRequest,
    PathRequest,
)
from machine_fleet_msgs.msg import (
    DeviceMode,
    FleetMachineState,
    MachineState,
    MachineRequest,
    StationRequest,
)
import socketio
import uvicorn
import yaml

app = FastAPI()


class Request(BaseModel):
    map_name: Optional[str] = None
    activity: Optional[str] = None
    activity_desc: Optional[dict] = None
    label: Optional[str] = None
    destination: Optional[dict] = None
    data: Optional[dict] = None
    speed_limit: Optional[float] = None
    toggle: Optional[bool] = None
    machine_desc: Optional[dict] = None
    station_desc: Optional[dict] = None


class Response(BaseModel):
    data: Optional[dict] = None
    success: bool
    msg: str


# Machine state:
class MCState:
    def __init__(self, state: MachineState = None) -> None:
        self.state = state


# ------------------------------------------------------------------------------
# Fleet Manager
# ------------------------------------------------------------------------------
class State:

    def __init__(self, state: RobotState = None, destination: Location = None):
        self.state = state
        self.destination = destination
        self.last_request = None
        self.last_completed_request = None
        self.perform_action_mode = False
        self.svy_transformer = Transformer.from_crs("EPSG:4326", "EPSG:3414")
        self.gps_pos = [0, 0]

    def gps_to_xy(self, gps_json: dict):
        svy21_xy = self.svy_transformer.transform(gps_json["lat"], gps_json["lon"])
        self.gps_pos[0] = svy21_xy[1]
        self.gps_pos[1] = svy21_xy[0]

    def is_expected_task_id(self, task_id):
        if self.last_request is not None:
            if task_id != self.last_request.task_id:
                return False
        return True


class DockInfo:
    def __init__(
        self,
        machine_name: str = None,
        response_state: str = None,
        dock_limit: DockLimit = None,
        tag_names: list[DockTag] = [],
        go_in_dock: list[DockParam] = [],
        go_out_dock: list[DockParam] = [],
    ) -> None:
        self.machine_name = machine_name
        self.response_state = response_state
        self.tag_names = tag_names
        self.dock_limit = dock_limit
        self.go_in_dock = go_in_dock
        self.go_out_dock = go_out_dock


def search_mode_docking(dock_name: str):
    match = re.search(r"--(.+)", dock_name)
    if match:
        result = match.group(1)
        return result
    else:
        return None


class FleetManager(Node):
    _dock_context: dict[str, DockInfo]
    robots: dict[str, State]
    machines: dict[str, MCState]

    def __init__(self, config, nav_graph):
        self.debug = False
        self.config = config
        self.fleet_name = self.config["rmf_fleet"]["name"]
        mgr_config = self.config["fleet_manager"]

        self.gps = False
        self.offset = [0, 0]
        reference_coordinates_yaml = mgr_config.get("reference_coordinates")
        if reference_coordinates_yaml is not None:
            offset_yaml = reference_coordinates_yaml.get("offset")
            if offset_yaml is not None and len(offset_yaml) > 1:
                self.gps = True
                self.offset = offset_yaml

        super().__init__(f"{self.fleet_name}_fleet_manager")

        self.robots = {}  # Map robot name to state
        self.action_paths = {}  # Map activities to paths
        self._dock_context = {}
        self.machines = {}  # Map machine name to state

        if "docks" in self.config:
            self.add_dock_context()
        assert len(self._dock_context) > 0

        # Add robot context:
        for robot_name, _ in self.config["rmf_fleet"]["robots"].items():
            self.robots[robot_name] = State()
        assert len(self.robots) > 0

        # Add machine context:
        self.add_machine_context(nav_graph)

        profile = traits.Profile(
            geometry.make_final_convex_circle(self.config["rmf_fleet"]["profile"]["footprint"]),
            geometry.make_final_convex_circle(self.config["rmf_fleet"]["profile"]["vicinity"]),
        )
        self.vehicle_traits = traits.VehicleTraits(
            linear=traits.Limits(*self.config["rmf_fleet"]["limits"]["linear"]),
            angular=traits.Limits(*self.config["rmf_fleet"]["limits"]["angular"]),
            profile=profile,
        )
        self.vehicle_traits.differential.reversible = self.config["rmf_fleet"]["reversible"]

        fleet_manager_config = self.config["fleet_manager"]
        self.action_paths = fleet_manager_config.get("action_paths", {})
        self.sio = socketio.Client()

        @self.sio.on("/gps")
        def message(data):
            try:
                robot = json.loads(data)
                robot_name = robot["robot_id"]
                self.robots[robot_name].gps_to_xy(robot)
            except KeyError as e:
                self.get_logger().info(f"Malformed GPS Message!: {e}")

        if self.gps:
            while True:
                try:
                    self.sio.connect("http://0.0.0.0:8080")
                    break
                except Exception:
                    self.get_logger().info(
                        "Trying to connect to sio server at " "http://0.0.0.0:8080.."
                    )
                    time.sleep(1)

        self.create_subscription(
            FleetState,
            "fleet_states",
            self.fleet_states_cb,
            100,
        )

        self.create_subscription(
            FleetMachineState,
            "fleet_machine_state",
            self.fleet_machine_state_cb,
            100,
        )

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL,
        )

        self.create_subscription(
            DockSummary,
            "dock_summary",
            self.dock_summary_cb,
            qos_profile=transient_qos,
        )

        self.path_pub = self.create_publisher(
            PathRequest,
            "robot_path_requests",
            qos_profile=qos_profile_system_default,
        )

        self.dock_pub = self.create_publisher(
            DockRequest, "robot_dock_requests", qos_profile=qos_profile_system_default
        )

        self.mode_pub = self.create_publisher(
            ModeRequest, "robot_mode_requests", qos_profile=qos_profile_system_default
        )

        self.cancel_pub = self.create_publisher(
            CancelRequest,
            "robot_cancel_requests",
            qos_profile=qos_profile_system_default,
        )

        # To publish robot localize request to Free Fleet Server
        self.localize_pub = self.create_publisher(
            LocalizeRequest,
            "robot_localize_requests",
            qos_profile=qos_profile_system_default,
        )

        self.machine_req_pub = self.create_publisher(
            MachineRequest,
            "adapter_machine_requests",
            qos_profile=qos_profile_system_default,
        )

        self.station_req_pub = self.create_publisher(
            StationRequest,
            "station_requests",
            qos_profile=qos_profile_system_default,
        )

        self.charger_req_pub = self.create_publisher(
            ChargerRequest, "charger_request", qos_profile=qos_profile_system_default
        )

        # ---------------------------- ROBOT --------------------------------- #
        @app.get("/open-rmf/rmf_vdm_fm/status/", response_model=Response)
        async def status(robot_name: Optional[str] = None):
            response = {"data": {}, "success": False, "msg": ""}
            if robot_name is None:
                response["data"]["all_robots"] = []
                for robot_name in self.robots:
                    state = self.robots.get(robot_name)
                    if state is None or state.state is None:
                        return response
                    response["data"]["all_robots"].append(self.get_robot_state(state, robot_name))
            else:
                state = self.robots.get(robot_name)
                if state is None or state.state is None:
                    return response
                response["data"] = self.get_robot_state(state, robot_name)
            response["success"] = True
            return response

        @app.post("/open-rmf/rmf_vdm_fm/navigate/", response_model=Response)
        async def navigate(robot_name: str, cmd_id: int, dest: Request):
            response = {"success": False, "msg": ""}
            if robot_name not in self.robots or len(dest.destination) < 1:
                return response

            robot = self.robots[robot_name]

            target_x = dest.destination["x"]
            target_y = dest.destination["y"]
            target_yaw = dest.destination["yaw"]
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
            # cur_loc = robot.state.location
            # path_request.path.append(cur_loc)

            disp = self.disp([target_x, target_y], [cur_x, cur_y])
            duration = int(disp / self.vehicle_traits.linear.nominal_velocity) + int(
                abs(abs(cur_yaw) - abs(target_yaw))
                / self.vehicle_traits.rotational.nominal_velocity
            )
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

            path_request.fleet_name = self.fleet_name
            path_request.robot_name = robot_name
            path_request.path.append(target_loc)
            path_request.task_id = str(cmd_id)
            self.path_pub.publish(path_request)

            if self.debug:
                print(f"Sending navigate request for {robot_name}: {cmd_id}")
            robot.last_request = path_request
            robot.destination = target_loc

            response["success"] = True
            return response

        @app.get("/open-rmf/rmf_vdm_fm/pause_robot/", response_model=Response)
        async def pause(robot_name: str, cmd_id: int):
            response = {"success": False, "msg": ""}
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
                print(f"Sending pause request for {robot_name}: {cmd_id}")
            robot.last_request = mode_request

            response["success"] = True
            return response

        @app.get("/open-rmf/rmf_vdm_fm/wait_robot/", response_model=Response)
        async def wait(robot_name: str, cmd_id: int):
            response = {"success": False, "msg": ""}
            if robot_name not in self.robots:
                return response

            robot = self.robots[robot_name]
            mode_request = ModeRequest()
            mode_request.fleet_name = self.fleet_name
            mode_request.robot_name = robot_name
            mode_request.mode.mode = RobotMode.MODE_WAITING

            mode_request.task_id = str(cmd_id)
            self.mode_pub.publish(mode_request)

            if self.debug:
                print(f"Sending wait request for {robot_name}: {cmd_id}")
            robot.last_request = mode_request

            response["success"] = True
            return response

        @app.get("/open-rmf/rmf_vdm_fm/resume_robot/", response_model=Response)
        async def resume(robot_name: str, cmd_id: int):
            response = {"success": False, "msg": ""}
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
                print(f"Sending resume request for {robot_name}: {cmd_id}")
            robot.last_request = mode_request

            response["success"] = True
            return response

        @app.get("/open-rmf/rmf_vdm_fm/stop_robot/", response_model=Response)
        async def stop(robot_name: str, cmd_id: int):
            response = {"success": False, "msg": ""}
            if robot_name not in self.robots:
                return response

            robot = self.robots[robot_name]
            cancel_request = CancelRequest()
            cancel_request.fleet_name = self.fleet_name
            cancel_request.robot_name = robot_name
            cancel_request.task_id = str(cmd_id)
            self.cancel_pub.publish(cancel_request)

            if self.debug:
                print(f"Sending stop request for {robot_name}: {cmd_id}")
            robot.last_request = cancel_request
            robot.destination = None

            response["success"] = True
            return response

        @app.get("/open-rmf/rmf_vdm_fm/action_paths/", response_model=Response)
        async def action_paths(activity: str, label: str):
            response = {"success": False, "msg": ""}
            if activity not in self.action_paths:
                return response

            if label not in self.action_paths[activity][label]:
                return response

            response["data"] = self.action_paths[activity][label]
            response["success"] = True
            return response

        @app.post("/open-rmf/rmf_vdm_fm/start_activity/", response_model=Response)
        async def start_activity(robot_name: str, cmd_id: int, request: Request):
            response = {"success": False, "msg": ""}
            if robot_name not in self.robots:
                return response

            robot = self.robots[robot_name]

            if request.activity == "dock":
                dock_request = DockRequest()
                if request.activity_desc["mode"] == "charge":
                    dock_request.dock_mode.mode = DockMode.MODE_CHARGE
                elif (
                    request.activity_desc["mode"] == "pickup"
                    or request.activity_desc["mode"] == "mpickup"
                ):
                    dock_request.dock_mode.mode = DockMode.MODE_PICKUP
                elif (
                    request.activity_desc["mode"] == "dropoff"
                    or request.activity_desc["mode"] == "mdropoff"
                ):
                    dock_request.dock_mode.mode = DockMode.MODE_DROPOFF
                elif request.activity_desc["mode"] == "undock":
                    dock_request.dock_mode.mode = DockMode.MODE_UNDOCK
                else:
                    response["msg"] = "Mode dock does not support. Please check mode!"
                    return response

                undock_dist = request.activity_desc.get("undock_dist", None)

                dock_config = self._dock_context.get(request.activity_desc["dock_name"], None)

                target_loc = Location()
                target_loc.level_name = request.map_name
                target_loc.x = request.activity_desc["location"]["x"]
                target_loc.y = request.activity_desc["location"]["y"]
                target_loc.yaw = request.activity_desc["location"]["yaw"]

                activity_path = {}
                activity_path.update({"map_name": request.map_name})
                activity_path.update(
                    {
                        "path": [
                            [
                                robot.state.location.x,
                                robot.state.location.y,
                                robot.state.location.yaw,
                            ],
                            [target_loc.x, target_loc.y, target_loc.yaw],
                        ]
                    }
                )
                dock_request.destination = target_loc
                dock_request.fleet_name = self.fleet_name
                dock_request.robot_name = robot_name
                dock_request.dock_name = request.activity_desc["dock_name"]

                if dock_request.dock_mode.mode == DockMode.MODE_UNDOCK and undock_dist is not None:
                    dockParam = DockParam()
                    dockParam.action_type = DockParam.TYPE_MOVE
                    dockParam.value = undock_dist
                    dock_request.go_out_dock.append(dockParam)
                elif dock_config is None:
                    response["msg"] = "Not found dock name in config!"
                    self.get_logger().warn(
                        f"Not found dock name [{request.activity_desc['dock_name']}] in config_file!"
                    )
                    return response
                else:
                    dock_request.dock_limit = dock_config.dock_limit
                    dock_request.tag_names = dock_config.tag_names
                    dock_request.go_in_dock = dock_config.go_in_dock
                    dock_request.go_out_dock = dock_config.go_out_dock

                dock_request.task_id = str(cmd_id)
                self.dock_pub.publish(dock_request)

                if self.debug:
                    print(
                        f"Sending [{request.activity}] at [{request.label}] "
                        f"request for {robot_name}: {cmd_id}"
                    )
                robot.last_request = dock_request
                robot.destination = target_loc

                response["success"] = True
                response["data"] = {}
                response["data"]["path"] = activity_path
                return response

        @app.post("/open-rmf/rmf_vdm_fm/localize/", response_model=Response)
        async def localize(robot_name: str, cmd_id: int, dest: Request):
            response = {"success": False, "msg": ""}
            if robot_name not in self.robots or len(dest.destination) < 1:
                return response

            robot = self.robots[robot_name]

            target_loc = Location()
            t = self.get_clock().now().to_msg()
            target_loc.level_name = dest.map_name
            target_loc.t = t
            target_loc.x = dest.destination["x"]
            target_loc.y = dest.destination["y"]
            target_loc.yaw = dest.destination["yaw"]

            localize_request = LocalizeRequest()
            localize_request.fleet_name = self.fleet_name
            localize_request.robot_name = robot_name
            localize_request.destination = target_loc

            localize_request.task_id = str(cmd_id)
            self.localize_pub.publish(localize_request)

            if self.debug:
                print(f"Sending localize request for {robot_name}: {cmd_id}")

            robot.last_request = localize_request
            robot.destination = target_loc
            response["success"] = True
            return response

        @app.post("/open-rmf/rmf_vdm_fm/toggle_teleop/", response_model=Response)
        async def toggle_teleop(robot_name: str, mode: Request):
            response = {"success": False, "msg": ""}
            if robot_name not in self.robots:
                return response
            # Toggle action mode
            self.robots[robot_name].perform_action_mode = mode.toggle
            response["success"] = True
            return response

        @app.post("/open-rmf/rmf_vdm_fm/toggle_attach/", response_model=Response)
        async def toggle_attach(robot_name: str, cmd_id: int, mode: Request):
            response = {"success": False, "msg": ""}
            if robot_name not in self.robots:
                return response
            # Toggle action mode
            if mode.toggle:
                # Use robot mode publisher to set it to "attaching cart mode"
                self.get_logger().info("Publishing attaching mode...")
                msg = self._make_mode_request(
                    robot_name, cmd_id, RobotMode.MODE_PERFORMING_ACTION, "attach_cart"
                )
            else:
                # Use robot mode publisher to set it to "detaching cart mode"
                self.get_logger().info("Publishing detaching mode...")
                msg = self._make_mode_request(
                    robot_name, cmd_id, RobotMode.MODE_PERFORMING_ACTION, "detach_cart"
                )
            self.mode_pub.publish(msg)
            response["success"] = True
            return response

        # ///////////////////////////////////////////////////////////////////////

        # ---------------------------- MACHINE --------------------------------- #
        @app.get("/open-rmf/rmf_vdm_fm/machine_status/", response_model=Response)
        async def machine_status(dock_name: str):
            response = {"data": {}, "success": False, "msg": ""}

            dock_config = self._dock_context.get(dock_name, None)
            if dock_config is None:
                response["msg"] = "dock_config not found. Please check dock_name!"
                return response

            machine_name = dock_config.machine_name
            if machine_name is None:
                response["msg"] = "machine_name not found. Please check dock_config!"
                return response
            else:
                state = self.machines.get(machine_name)
                if state is None or state.state is None:
                    return response
                response["data"] = self.get_machine_state(state, machine_name)
            response["success"] = True
            return response

        @app.post("/open-rmf/rmf_vdm_fm/machine_request/", response_model=Response)
        async def machine_request(dock_name: str, task: Request):
            response = {"success": False, "msg": ""}

            dock_config = self._dock_context.get(dock_name, None)
            if dock_config is None:
                return response

            machine_name = dock_config.machine_name

            machine_request = MachineRequest()
            if task.machine_desc["request_type"] == "dispenser":
                machine_request.request_type = MachineRequest.REQUEST_DISPENSER
            elif task.machine_desc["request_type"] == "ingestor":
                machine_request.request_type = MachineRequest.REQUEST_INGESTOR
            else:
                response["msg"] = (
                    "request_type machine does not support. Please check request_type!"
                )
                return response

            if task.machine_desc["mode"] == DeviceMode.MODE_IDLE:
                machine_request.request_mode.mode = DeviceMode.MODE_IDLE
            elif task.machine_desc["mode"] == DeviceMode.MODE_ACCEPT_DOCKIN:
                machine_request.request_mode.mode = DeviceMode.MODE_ACCEPT_DOCKIN
            elif task.machine_desc["mode"] == DeviceMode.MODE_ROBOT_DOCKED_IN:
                machine_request.request_mode.mode = DeviceMode.MODE_ROBOT_DOCKED_IN
            elif task.machine_desc["mode"] == DeviceMode.MODE_ACCEPT_DOCKOUT:
                machine_request.request_mode.mode = DeviceMode.MODE_ACCEPT_DOCKOUT
            elif task.machine_desc["mode"] == DeviceMode.MODE_CANCEL:
                machine_request.request_mode.mode = DeviceMode.MODE_CANCEL
            elif task.machine_desc["mode"] == DeviceMode.MODE_ROBOT_ERROR:
                machine_request.request_mode.mode = DeviceMode.MODE_ROBOT_ERROR
            else:
                response["msg"] = "Mode device request does not support. Please check mode!"
                return response

            machine_request.machine_name = machine_name
            machine_request.time = self.get_clock().now().to_msg()
            self.machine_req_pub.publish(machine_request)

            if self.debug:
                print(f"Sending adapter machine request for {machine_name}")

            response["success"] = True
            return response

        @app.post("/open-rmf/rmf_vdm_fm/station_request/", response_model=Response)
        async def station_request(station_name: str, task: Request):
            response = {"success": False, "msg": ""}

            dock_config = self._dock_context.get(station_name, None)
            if dock_config is None:
                return response

            station_request = StationRequest()
            if task.station_desc["station_type"] == "pickup":
                station_request.station_type = StationRequest.TYPE_PICKUP
            elif task.station_desc["station_type"] == "dropoff":
                station_request.station_type = StationRequest.TYPE_DROPOFF
            else:
                response["msg"] = "station_type does not support. Please check station_type!"
                return response

            if task.station_desc["mode"] == StationRequest.MODE_EMPTY:
                station_request.mode = StationRequest.MODE_EMPTY
            elif task.station_desc["mode"] == StationRequest.MODE_FILLED:
                station_request.mode = StationRequest.MODE_FILLED
            else:
                response["msg"] = "Mode station request does not support. Please check mode!"
                return response

            if dock_config.response_state is None:
                station_request.station_name = station_name
            else:
                station_request.station_name = dock_config.response_state
                station_request.station_type = StationRequest.TYPE_PICKUP

            station_request.time = self.get_clock().now().to_msg()
            self.station_req_pub.publish(station_request)

            if self.debug:
                print(f"Sending adapter station request for {station_name}")

            response["success"] = True
            return response

        # ///////////////////////////////////////////////////////////////////////

        # ---------------------------- CHARGER --------------------------------- #
        @app.post("/open-rmf/rmf_vdm_fm/charger_trigger/", response_model=Response)
        async def charger_trigger(robot_name: str, cmd_id: int, request: Request):
            response = {"success": False, "msg": ""}

            charger_request = ChargerRequest()
            if request.activity_desc["mode"] == "charge":
                charger_request.charger_mode.mode = ChargerMode.MODE_CHARGE
            elif request.activity_desc["mode"] == "uncharge":
                charger_request.charger_mode.mode = ChargerMode.MODE_UNCHARGE
            else:
                response["msg"] = "Mode charger does not support. Please check mode!"
                return response

            charger_request.fleet_name = self.fleet_name
            charger_request.charger_name = request.activity_desc["charger_name"]
            charger_request.robot_name = robot_name
            charger_request.request_id = str(cmd_id)

            self.charger_req_pub.publish(charger_request)

            if self.debug:
                print(
                    f'Sending charger request for {request.activity_desc["charger_name"]}: {cmd_id}'
                )

            response["success"] = True
            return response

        # ///////////////////////////////////////////////////////////////////////

    def add_dock_context(self):
        for dock in self.config["docks"]:
            dockConf = self.config["docks"][dock]
            machine_name = dockConf.get("machine_name", None)
            response_state = dockConf.get("response_state", None)
            tag_names_conf = dockConf.get("tag_names", [])
            go_in_dock_conf = dockConf.get("go_in_dock", None)
            go_out_dock_conf = dockConf.get("go_out_dock", None)
            dock_limit_conf = dockConf.get("limits", None)

            dock_limit = DockLimit()
            if dock_limit_conf is not None:
                dock_limit.rotate_angle = dock_limit_conf.get("rotate_angle", 0)
                dock_limit.rotate_orientation = dock_limit_conf.get("rotate_orientation", 0)

            assert go_out_dock_conf is not None, f"dock [{dock}] need config go_out_dock!"

            # add tag names
            tag_names = []
            for tag in tag_names_conf:
                dockTag = DockTag()
                dockTag.name = tag
                tag_names.append(dockTag)

            # add action of go_in_dock
            go_in_dock = []
            if go_in_dock_conf is not None:
                for action_conf in go_in_dock_conf:
                    action = action_conf.split(":")
                    assert (
                        action[0] == "rot" or action[0] == "mov"
                    ), f"dock [{dock}] - go_in_dock: {action} invalid, format (mov:value or rot:value)!"
                    param = DockParam()
                    if action[0] == "rot":
                        param.action_type = DockParam.TYPE_ROTATE
                    else:
                        param.action_type = DockParam.TYPE_MOVE
                    param.value = float(action[1])
                    go_in_dock.append(param)
            else:
                go_in_dock.append(DockParam())

            # add action of go_out_dock
            go_out_dock = []
            for action_conf in go_out_dock_conf:
                action = action_conf.split(":")
                assert (
                    action[0] == "rot" or action[0] == "mov"
                ), f"dock [{dock}] - go_out_dock: {action} invalid, format (mov:value or rot:value)!"
                param = DockParam()
                if action[0] == "rot":
                    param.action_type = DockParam.TYPE_ROTATE
                else:
                    param.action_type = DockParam.TYPE_MOVE
                param.value = float(action[1])
                go_out_dock.append(param)

            dInf = DockInfo(
                machine_name,
                response_state,
                dock_limit,
                tag_names,
                go_in_dock,
                go_out_dock,
            )
            self._dock_context.update({dock: dInf})
        return

    def add_machine_context(self, nav_graph):
        for level in nav_graph["levels"]:
            for wp in nav_graph["levels"][level]["vertices"]:
                assert len(wp) == 3, "Vertical structure not match, please check!"

                machine_name = None
                if (
                    "pickup_dispenser" in wp[2]
                    and search_mode_docking(wp[2]["dock_name"]) == "mpickup"
                ):
                    machine_name = wp[2]["pickup_dispenser"]
                elif (
                    "dropoff_ingestor" in wp[2]
                    and search_mode_docking(wp[2]["dock_name"]) == "mdropoff"
                ):
                    machine_name = wp[2]["dropoff_ingestor"]

                if machine_name is not None and machine_name not in self.machines:
                    self.machines[machine_name] = MCState()
        return

    def _make_mode_request(self, robot_name, cmd_id, mode, action=""):
        mode_msg = ModeRequest()
        mode_msg.fleet_name = self.fleet_name
        mode_msg.robot_name = robot_name
        mode_msg.mode.mode = mode
        mode_msg.mode.mode_request_id = cmd_id
        mode_msg.mode.performing_action = action
        return mode_msg

    def fleet_states_cb(self, msg: FleetState):
        if msg.name == self.fleet_name:
            dataRobot = msg.robots
            for robotMsg in dataRobot:
                if robotMsg.name in self.robots:
                    robot = self.robots[robotMsg.name]
                    if (
                        not robot.is_expected_task_id(robotMsg.task_id)
                        and not robot.perform_action_mode
                    ):
                        # This message is out of date, so disregard it.
                        if robot.last_request is not None:
                            # Resend the latest task request for this robot, in case
                            # the message was dropped.
                            if not self.debug:
                                print(
                                    f"Republishing task request for {robotMsg.name}: "
                                    f"{robot.last_request.task_id}, "
                                    f"because it is currently following {robotMsg.task_id}"
                                )
                            if type(robot.last_request) is PathRequest:
                                self.path_pub.publish(robot.last_request)
                            elif type(robot.last_request) is ModeRequest:
                                self.mode_pub.publish(robot.last_request)
                            elif type(robot.last_request) is DockRequest:
                                self.dock_pub.publish(robot.last_request)
                            elif type(robot.last_request) is CancelRequest:
                                self.cancel_pub.publish(robot.last_request)
                            elif type(robot.last_request) is LocalizeRequest:
                                self.localize_pub.publish(robot.last_request)
                        continue

                    robot.state = robotMsg
                    # Check if robot has reached destination
                    if robot.destination is None:
                        continue

                    if (
                        (
                            robotMsg.mode.mode == RobotMode.MODE_IDLE
                            or robotMsg.mode.mode == RobotMode.MODE_CHARGING
                        )
                        and len(robotMsg.path) == 0
                        and robotMsg.task_id
                    ):
                        if type(robot.last_request) is LocalizeRequest:
                            if (
                                robot.last_request.destination.level_name
                                != robotMsg.location.level_name
                            ):
                                continue

                        robot.destination = None
                        completed_request = int(robotMsg.task_id)
                        if robot.last_completed_request != completed_request:
                            if self.debug:
                                print(
                                    f"Detecting completed request for {robotMsg.name}: "
                                    f"{completed_request}"
                                )
                        robot.last_completed_request = completed_request
                else:
                    self.get_logger().warn(
                        f'Detect robot "{robotMsg.name}" is not in config file, pleascheck!'
                    )

    def fleet_machine_state_cb(self, msg: FleetMachineState):
        dataMachine = msg.machines
        machineMsg: MachineState
        for machineMsg in dataMachine:
            if machineMsg.machine_name in self.machines:
                machine = self.machines[machineMsg.machine_name]
                machine.state = machineMsg

    def dock_summary_cb(self, msg):
        for fleet in msg.docks:
            if fleet.fleet_name == self.fleet_name:
                for dock in fleet.params:
                    self.docks[dock.start] = dock.path

    def get_robot_state(self, robot: State, robot_name):
        data = {}
        if self.gps:
            position = copy.deepcopy(robot.gps_pos)
        else:
            position = [robot.state.location.x, robot.state.location.y]
        angle = robot.state.location.yaw
        data["robot_name"] = robot_name
        data["map_name"] = robot.state.location.level_name
        data["position"] = {"x": position[0], "y": position[1], "yaw": angle}
        data["battery"] = robot.state.battery_percent
        if robot.destination is not None and robot.last_request is not None:
            destination = robot.destination
            # remove offset for calculation if using gps coords
            if self.gps:
                position[0] -= self.offset[0]
                position[1] -= self.offset[1]
            # calculate arrival estimate
            dist_to_target = self.disp(position, [destination.x, destination.y])
            ori_delta = abs(abs(angle) - abs(destination.yaw))
            if ori_delta > np.pi:
                ori_delta = ori_delta - (2 * np.pi)
            if ori_delta < -np.pi:
                ori_delta = (2 * np.pi) + ori_delta
            duration = (
                dist_to_target / self.vehicle_traits.linear.nominal_velocity
                + ori_delta / self.vehicle_traits.rotational.nominal_velocity
            )
            cmd_id = int(robot.last_request.task_id)
            data["destination_arrival"] = {
                "cmd_id": cmd_id,
                "duration": duration,
            }
        else:
            data["destination_arrival"] = None

        data["last_completed_request"] = robot.last_completed_request
        if (
            # robot.state.mode.mode == RobotMode.MODE_WAITING
            robot.state.mode.mode
            == RobotMode.MODE_ADAPTER_ERROR
        ):
            # The name of MODE_WAITING is not very intuitive, but the slotcar
            # plugin uses it to indicate when another robot is blocking its
            # path.
            #
            # MODE_ADAPTER_ERROR means the robot received a plan that
            # didn't make sense, i.e. the plan expected the robot was starting
            # very far from its real present location. When that happens we
            # should replan, so we'll set replan to true in that case as well.
            data["replan"] = True
        else:
            data["replan"] = False

        data["mode"] = robot.state.mode.mode
        return data

    def get_machine_state(self, machine: MCState, machine_name):
        data = {}
        data["machine_name"] = machine_name
        data["dispenser_mode"] = machine.state.dispenser_mode.mode
        data["ingestor_mode"] = machine.state.ingestor_mode.mode
        data["mode"] = machine.state.machine_mode
        return data

    def disp(self, A, B):
        return math.sqrt((A[0] - B[0]) ** 2 + (A[1] - B[1]) ** 2)


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
        description="Configure and spin up the fleet adapter",
    )
    parser.add_argument(
        "-c",
        "--config_file",
        type=str,
        required=True,
        help="Path to the config.yaml file",
    )
    parser.add_argument(
        "-n",
        "--nav_graph",
        type=str,
        required=True,
        help="Path to the nav_graph for this fleet manager",
    )
    args = parser.parse_args(args_without_ros[1:])
    print("Starting fleet manager...")

    config_path = args.config_file
    nav_graph_path = args.nav_graph

    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    with open(nav_graph_path, "r") as f:
        nav_graph = yaml.safe_load(f)

    fleet_manager = FleetManager(config, nav_graph)

    spin_thread = threading.Thread(target=rclpy.spin, args=(fleet_manager,))
    spin_thread.start()

    uvicorn.run(
        app,
        host=config["fleet_manager"]["ip"],
        port=config["fleet_manager"]["port"],
        log_level="warning",
    )


if __name__ == "__main__":
    main(sys.argv)
