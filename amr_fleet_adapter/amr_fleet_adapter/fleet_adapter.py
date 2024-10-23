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
import asyncio
import math
import sys
import threading
import time
import re

import rclpy
from rclpy.duration import Duration
import rclpy.node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy as Reliability
import rmf_adapter
from rmf_adapter import Adapter
import rmf_adapter.easy_full_control as rmf_easy
from rmf_fleet_msgs.msg import ClosedLanes
from rmf_fleet_msgs.msg import LaneRequest
from rmf_fleet_msgs.msg import ModeRequest
from rmf_fleet_msgs.msg import RobotMode
import yaml

from .RobotClientAPI import RobotAPI
from .RobotClientAPI import RobotAPIResult
from .RobotClientAPI import RobotUpdateData


class VertexInfo:
    def __init__(self, orientation: float, in_lift: bool) -> None:
        self.orientation = orientation
        self.in_lift = in_lift


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def main(argv=sys.argv):
    # Init rclpy and adapter
    rclpy.init(args=argv)
    rmf_adapter.init_rclcpp()
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
        help="Path to the nav_graph for this fleet adapter",
    )
    parser.add_argument(
        "-sim",
        "--use_sim_time",
        action="store_true",
        help="Use sim time, default: false",
    )
    args = parser.parse_args(args_without_ros[1:])
    print("Starting fleet adapter...")

    config_path = args.config_file
    nav_graph_path = args.nav_graph

    fleet_config = rmf_easy.FleetConfiguration.from_config_files(
        config_path, nav_graph_path
    )
    assert fleet_config, f"Failed to parse config file [{config_path}]"

    # Parse the yaml in Python to get the fleet_manager info
    with open(config_path, "r") as f:
        config_yaml = yaml.safe_load(f)

    # ROS 2 node for the command handle
    fleet_name = fleet_config.fleet_name
    node = rclpy.node.Node(f"{fleet_name}_command_handle")
    adapter = Adapter.make(f"{fleet_name}_fleet_adapter")
    assert adapter, (
        "Unable to initialize fleet adapter. "
        "Please ensure RMF Schedule Node is running"
    )

    # Enable sim time for testing offline
    if args.use_sim_time:
        param = Parameter("use_sim_time", Parameter.Type.BOOL, True)
        node.set_parameters([param])
        adapter.node.use_sim_time()

    adapter.start()
    time.sleep(1.0)

    node.declare_parameter("server_uri", "")
    server_uri = node.get_parameter("server_uri").get_parameter_value().string_value
    if server_uri == "":
        server_uri = None

    fleet_config.server_uri = server_uri
    fleet_handle = adapter.add_easy_fleet(fleet_config)
    fleet_handle.more().fleet_state_publish_period(None)

    # Initialize robot API for this fleet
    fleet_mgr_yaml = config_yaml["fleet_manager"]
    vertexs_yaml = config_yaml["vertexs"]
    update_period = 1.0 / fleet_mgr_yaml.get("robot_state_update_frequency", 10.0)
    api_prefix = "http://" + fleet_mgr_yaml["ip"] + ":" + str(fleet_mgr_yaml["port"])
    api = RobotAPI(api_prefix, fleet_mgr_yaml["user"], fleet_mgr_yaml["password"])

    # Trạm sạc trigger qua server hay không:
    charger_server = config_yaml["rmf_fleet"]["charger_server_running"]

    # Cấu hình thông tin cho các vertexs đặc biệt:
    vertexs_config = {}
    for vertex in vertexs_yaml:
        if type(vertexs_yaml[vertex]) == list:
            vertexs_config.update(
                {
                    str(vertex): VertexInfo(
                        vertexs_yaml[vertex][0], vertexs_yaml[vertex][1]
                    )
                }
            )
        else:
            vertexs_config.update(
                {str(vertex): VertexInfo(vertexs_yaml[vertex], False)}
            )
    robots: dict[str, RobotAdapter]
    robots = {}
    for robot_name in fleet_config.known_robots:
        robot_config = fleet_config.get_known_robot_configuration(robot_name)
        robots[robot_name] = RobotAdapter(
            robot_name,
            robot_config,
            node,
            api,
            fleet_handle,
            vertexs_config,
            charger_server,
        )

    async def state_updates():
        robot_updaters = []
        for robot in robots.values():
            robot_updaters.append(robot.update_loop(update_period))
        await asyncio.gather(*robot_updaters)

    def update_loop():
        event_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(event_loop)
        event_loop.run_until_complete(state_updates())

    update_thread = threading.Thread(target=update_loop, args=())
    update_thread.start()

    # Connect to the extra ROS2 topics that are relevant for the adapter
    ros_connections(node, robots, fleet_handle)

    # Create executor for the command handle node
    rclpy_executor = rclpy.executors.SingleThreadedExecutor()
    rclpy_executor.add_node(node)

    # Start the fleet adapter
    rclpy_executor.spin()

    # Shutdown
    node.destroy_node()
    rclpy_executor.shutdown()
    rclpy.shutdown()


def search_mode_docking(dock_name: str):
    match = re.search(r"--(.+)", dock_name)
    if match:
        result = match.group(1)
        return result
    else:
        return None


# Parallel processing solution derived from
# https://stackoverflow.com/a/59385935
def parallel(f):
    def run_in_parallel(*args, **kwargs):
        return asyncio.get_event_loop().run_in_executor(None, f, *args, **kwargs)

    return run_in_parallel


class MissionHandle:
    def __init__(
        self,
        execution,
        navigate=False,
        docking=False,
        localize=False,
        charger=None,
        destination=None,
    ):
        self.execution = execution
        self.navigate = navigate
        self.docking = docking
        self.localize = localize
        self.charger = charger
        self.destination = destination
        self.done = False
        # self.mission_queue_id = None
        # self.mutex = threading.Lock()
        # Block before beginning the request to guarantee that a call to stop()
        # cannot possibly lock it first
        # self.mutex.acquire(blocking=True)

    # def set_mission_queue_id(self, mission_queue_id):
    #     self.mission_queue_id = mission_queue_id
    #     self.mutex.release()

    @property
    def activity(self):
        # Move the execution reference into a separate variable just in case
        # another thread modifies self.execution while we're still using it.
        execution = self.execution
        if execution is not None and not self.done:
            return execution.identifier
        return None


class RobotAdapter:
    def __init__(
        self,
        name: str,
        configuration,
        node,
        api: RobotAPI,
        fleet_handle,
        vertexs_config: dict[str, VertexInfo],
        charger_server: bool,
    ):
        self.name = name
        self.execution = None
        self.mission: MissionHandle | None = None
        self.teleoperation = None
        self.cmd_id = 0
        self.update_handle = None
        self.configuration = configuration
        self.node = node
        self.api = api
        self.fleet_handle = fleet_handle
        self.override = None

        self.disconnect = False
        self.paused = False
        self.paused_mission: MissionHandle | None = None
        self.vertexs_config = vertexs_config
        self.charger_server = charger_server
        self.undock = False
        self.unlift = False

        # Threading variables
        self._lock = threading.Lock()
        self.issue_cmd_thread = None
        self.cancel_cmd_event = threading.Event()

        self.last_known_status = None

    @property
    def activity(self):
        # Move the mission reference into a separate variable just in case
        # another thread modifies self.mission while we're still using it.
        mission = self.mission
        if mission is not None:
            return self.mission.activity
        return None

    async def update_loop(self, period):
        while rclpy.ok():
            now = self.node.get_clock().now()
            next_wakeup = now + Duration(nanoseconds=period * 1e9)
            data = self.api.get_data(self.name)
            if data is None:
                if not self.disconnect:
                    self.disconnect = True
                    self.node.get_logger().warn(
                        f"Unable to retrieve state from robot [{self.name}]!"
                    )
            else:
                if self.disconnect:
                    self.node.get_logger().info(
                        f"Robot [{self.name}] connectivity resumed, received "
                        f"status from robot successfully."
                    )
                    self.disconnect = False

                state = rmf_easy.RobotState(data.map, data.position, data.battery_soc)

                if self.update_handle is None:
                    self.last_known_status = data
                    self.update_handle = self.fleet_handle.add_robot(
                        self.name, state, self.configuration, self.make_callbacks()
                    )
                else:
                    await self.update(state, data)
            while self.node.get_clock().now() < next_wakeup:
                # time.sleep(0.01)
                await asyncio.sleep(0.01)

    @parallel
    def update(self, state, data):
        with self._lock:
            # Update the stored mission status from AMR
            mission = self.mission
            self.update_mission_status(data, mission)

            # Update RMF with the latest RobotState
            if (
                mission is None
                or (mission.localize and mission.done)
                or not mission.localize
            ):
                self.update_handle.update(state, self.activity)
                self.last_known_status = data
            else:
                self.node.get_logger().info(
                    f"Mission is None / Robot is localizing, ignore status " f"update"
                )

            # Update RMF to mark the ActionExecution as finished
            if mission is not None:
                if mission.done:
                    self.update_rmf_finished(mission)
                elif (
                    data.mode == RobotMode.MODE_EMERGENCY
                    or data.mode == RobotMode.MODE_REQUEST_ERROR
                ):
                    if data.mode == RobotMode.MODE_REQUEST_ERROR:
                        self.node.get_logger().error(
                            f"Robot {self.name} has ERROR when process last requested "
                            f"with task_id: {self.update_handle.more().current_task_id()}."
                        )
                    elif data.mode == RobotMode.MODE_EMERGENCY:
                        self.node.get_logger().error(
                            f"Robot {self.name} has EMERGENCY_STOP when process last requested "
                            f"with task_id: {self.update_handle.more().current_task_id()}."
                        )

                    self.update_handle.more().kill_task(
                        self.update_handle.more().current_task_id(),
                        ["kill_task"],
                        self.on_kill,
                    )

                    mission = None

            if self.teleoperation is not None:
                self.teleoperation.update(data)

    def on_kill(self, killed):
        self.node.get_logger().info(f"on_kill result {killed} ")
        if not killed:
            self.node.get_logger().error(f"on_kill with error")

    def is_charging(self, status: RobotUpdateData):
        # Note: Not the best way to verify that robot is charging but there's
        # currently no other option
        if status.mode == RobotMode.MODE_CHARGING:
            return True
        return False

    def update_rmf_finished(self, mission: MissionHandle):
        if not mission.execution:
            return
        mission.execution.finished()
        mission.execution = None

    def update_mission_status(self, status: RobotUpdateData, mission: MissionHandle):
        if mission is None:
            # Either we don't have a mission or we don't know the
            # mission_queue_id yet so just continue as normal for now
            return

        if mission.execution is None:
            # This mission is already finished, so we return early
            return

        if status.is_command_completed(self.cmd_id):
            if mission.charger is not None and self.is_charging(status):
                self.node.get_logger().info(
                    f"Robot [{self.name}] has begun charging..."
                )
            elif mission.docking:
                self.undock = True
                mission.docking = False
                self.node.get_logger().info(f"Robot [{self.name}] has docked finished.")

                if (
                    mission.destination.name
                    and search_mode_docking(mission.destination.name) == "charge"
                    and self.charger_server
                ):
                    chargerName = mission.destination.name.split("--")[0]
                    self.node.get_logger().info(
                        f"Trigger [{chargerName}] for robot [{self.name}] to charge."
                    )
                    process = {
                        "charger_name": chargerName,
                        "mode": "charge",
                    }
                    self.attempt_cmd_until_success(
                        cmd=self.api.charger_trigger,
                        args=(
                            self.name,
                            self.cmd_id,
                            process,
                        ),
                    )

            elif mission.localize:
                mission.localize = False
                self.node.get_logger().info(
                    f"Robot [{self.name}] has relocalize to [{mission.destination.map}] success!"
                )
            elif mission.navigate:
                mission.navigate = False
                self.node.get_logger().info(
                    f"Robot [{self.name}] has navigated finished (inside_lift: {mission.destination.inside_lift})."
                )
                if mission.destination.inside_lift:
                    self.unlift = True

            mission.done = True
        else:
            if (
                status.mode == RobotMode.MODE_REQUEST_ERROR
                or status.mode == RobotMode.MODE_EMERGENCY
            ):
                self.unlift = False
                self.undock = False
                self.paused = False

    def make_callbacks(self):
        callbacks = rmf_easy.RobotCallbacks(
            lambda destination, execution: self.navigate(destination, execution),
            lambda activity: self.stop(activity),
            lambda category, description, execution: self.execute_action(
                category, description, execution
            ),
        )
        callbacks.localize = lambda estimate, execution: self.localize(
            estimate, execution
        )
        return callbacks

    def navigate(self, destination, execution):
        with self._lock:
            # self.execution = execution
            self.node.get_logger().info(
                f"Commanding [{self.name}] to navigate to {destination.position} "
                f"on map [{destination.map}]: cmd_id {self.cmd_id}"
            )

            # If the nav command coming in is to bring the robot to same waypoint
            # with waypoint robot is at on, we ignore this nav command
            if (
                self.last_known_status is not None
                and self.dist(self.last_known_status.position[0:2], destination.xy)
                <= 0.25
            ):

                self.node.get_logger().info(
                    f"[{self.name}] Received navigation command to waypoint but "
                    f"robot is already at the same waypoint, ignoring command and "
                    f"marking it as finished."
                )
                if destination.dock is not None:
                    self.undock = True

                self.mission = MissionHandle(execution, destination=destination)
                return

            self.cmd_id += 1
            # Check if robot need undock:
            if self.undock:
                self.undock = False
                if self.mission is not None:
                    destinationUndock = self.mission.destination
                    self.mission = MissionHandle(execution, destination=destination)
                    self.node.get_logger().info(
                        f"[{self.name}] Received navigation command but "
                        f"robot will undock first."
                    )
                    self.attempt_cmd_until_success(
                        cmd=self.perform_docking,
                        args=(
                            destinationUndock,
                            "undock",
                        ),
                    )
                    return
                else:
                    self.node.get_logger().warn(
                        f"[{self.name}] need undock firt but mission information is not found!"
                    )
            # Check if robot need unlift:
            elif self.unlift:
                self.mission = MissionHandle(execution, destination=destination)
                self.unlift = False
                unliftDis = -self.dist(
                    self.last_known_status.position[0:2], destination.xy
                )
                self.node.get_logger().info(
                    f"[{self.name}] Received navigation command but "
                    f"robot will unlift: {unliftDis} first."
                )
                self.attempt_cmd_until_success(
                    cmd=self.perform_docking,
                    args=(
                        destination,
                        "undock",
                        unliftDis,
                    ),
                )
                return
            # Check if robot need docking
            elif destination.dock is not None:
                self.node.get_logger().info(
                    f"[{self.name}] Received navigation command to "
                    f"dock, will trigger docking to '{destination.name}'"
                )
                self.mission = MissionHandle(
                    execution, docking=True, destination=destination
                )
                self.attempt_cmd_until_success(
                    cmd=self.perform_docking, args=(destination,)
                )
                return

            # Navigation normal:
            self.mission = MissionHandle(
                execution, navigate=True, destination=destination
            )
            vertex = None
            if destination.name != "":
                vertex = self.vertexs_config.get(destination.name, None)

            if vertex != None:
                pose = [
                    destination.position[0],
                    destination.position[1],
                    vertex.orientation,
                ]
            else:
                pose = destination.position

            self.attempt_cmd_until_success(
                cmd=self.api.navigate,
                args=(
                    self.name,
                    self.cmd_id,
                    pose,
                    destination.map,
                    destination.speed_limit,
                ),
            )

    def localize(self, estimate, execution):
        with self._lock:
            self.cmd_id += 1
            self.mission = MissionHandle(execution, localize=True, destination=estimate)
            self.node.get_logger().info(
                f"Commanding [{self.name}] to localize to {estimate.name} "
                f"on map [{estimate.map}]: cmd_id {self.cmd_id}"
            )
            if estimate.inside_lift:
                lift_name = estimate.inside_lift.name
                self.node.get_logger().info(
                    f"[{self.name}] in lift with lift_name: {lift_name}"
                )
            self.attempt_cmd_until_success(
                cmd=self.api.localize,
                args=(self.name, self.cmd_id, estimate.map, estimate.position),
            )

    def pause(self):
        with self._lock:
            """Set pause flag and hold on to any requested navigate"""
            mission = self.mission
            if mission is not None and mission.execution is not None:
                if not self.paused:
                    self.paused = True
                    self.cmd_id += 1
                    # self.paused_mission = mission
                    self.node.get_logger().info(
                        f"[PAUSE] {self.name}: current mission saved!"
                    )
                    self.attempt_cmd_until_success(
                        cmd=self.api.pause, args=(self.name, self.cmd_id)
                    )
                else:
                    self.node.get_logger().info(
                        f"[PAUSE] {self.name}: robot was paused!"
                    )
            else:
                self.node.get_logger().info(
                    f"{self.name}: receive paused action but robot don't have mission!"
                )

    def resume(self):
        with self._lock:
            """Unset pause flag and substitute paused mission if no paths exist."""
            if self.paused:
                self.cmd_id += 1
                self.attempt_cmd_until_success(
                    cmd=self.api.resume, args=(self.name, self.cmd_id)
                )
                self.paused = False
                # self.mission = self.paused_mission
                self.node.get_logger().info(
                    f"[RESUME] {self.name}: saved mission restored!"
                )

    def stop(self, activity):
        with self._lock:
            self.cmd_id += 1
            mission = self.mission
            if mission is not None:
                if mission.execution is not None and activity.is_same(
                    mission.execution.identifier
                ):
                    self.node.get_logger().info(
                        f"[{self.name}] Stop requested from RMF!"
                    )
                    self.attempt_cmd_until_success(
                        cmd=self.api.stop, args=(self.name, self.cmd_id)
                    )
                    self.mission = None
                    self.paused = False

    def execute_action(self, category: str, description: dict, execution):
        self.cmd_id += 1
        self.execution = execution

        match category:
            case "teleop":
                self.teleoperation = Teleoperation(execution)
                self.attempt_cmd_until_success(
                    cmd=self.api.toggle_teleop, args=(self.name, True)
                )

    def finish_action(self):
        # This is triggered by a ModeRequest callback which allows human
        # operators to manually change the operational mode of the robot. This
        # is typically used to indicate when teleoperation has finished.
        if self.execution is not None:
            self.execution.finished()
            self.execution = None
            self.attempt_cmd_until_success(
                cmd=self.api.toggle_teleop, args=(self.name, False)
            )

    def perform_docking(
        self, destination, mode: str | None = None, unlift: float | None = None
    ):
        if mode is not None:
            dock_mode = mode
        else:
            dock_mode = search_mode_docking(dock_name=destination.name)
        if dock_mode is None:
            self.node.get_logger().error(
                f"Can't find mode dock in dock_name: {self.dock_name}. "
                f"Please ensure, dock_name liked that *****--mode_dock!"
            )
            return False

        location = {
            "x": destination.position[0],
            "y": destination.position[1],
            "yaw": destination.position[2],
        }

        activity_des = {
            "mode": dock_mode,
            "dock_name": destination.name,
            "location": location,
        }

        if unlift is not None:
            activity_des.update({"unlift": unlift})

        match self.api.start_activity(
            self.name, self.cmd_id, "dock", activity_des, destination.map
        ):
            case (RobotAPIResult.SUCCESS, path):
                # self.override = self.mission.execution.override_schedule(
                #     path["map_name"], path["path"]
                # )
                return True
            case RobotAPIResult.RETRY:
                return False
            case RobotAPIResult.IMPOSSIBLE:
                # If the fleet manager does not know this dock name, then treat
                # it as a regular navigation request
                return self.api.navigate(
                    self.name,
                    self.cmd_id,
                    destination.position,
                    destination.map,
                    destination.speed_limit,
                )

    def attempt_cmd_until_success(self, cmd, args):
        self.cancel_cmd_attempt()

        def loop():
            while not cmd(*args):
                self.node.get_logger().warn(
                    f"Failed to contact fleet manager for robot {self.name}"
                )
                if self.cancel_cmd_event.wait(1.0):
                    break

        self.issue_cmd_thread = threading.Thread(target=loop, args=())
        self.issue_cmd_thread.start()

    def cancel_cmd_attempt(self):
        if self.issue_cmd_thread is not None:
            self.cancel_cmd_event.set()
            if self.issue_cmd_thread.is_alive():
                self.issue_cmd_thread.join()
                self.issue_cmd_thread = None
        self.cancel_cmd_event.clear()

    def dist(self, A, B):
        assert len(A) > 1
        assert len(B) > 1
        return math.sqrt((A[0] - B[0]) ** 2 + (A[1] - B[1]) ** 2)


class Teleoperation:

    def __init__(self, execution):
        self.execution = execution
        self.override = None
        self.last_position = None

    def update(self, data: RobotUpdateData):
        if self.last_position is None:
            print("about to override schedule with " f"{data.map}: {[data.position]}")
            self.override = self.execution.override_schedule(
                data.map, [data.position], 30.0
            )
            self.last_position = data.position
        else:
            dx = self.last_position[0] - data.position[0]
            dy = self.last_position[1] - data.position[1]
            dist = math.sqrt(dx * dx + dy * dy)
            if dist > 0.1:
                print("about to replace override schedule")
                self.override = self.execution.override_schedule(
                    data.map, [data.position], 30.0
                )
                self.last_position = data.position


# @parallel
# def update_robot(robot: RobotAdapter):
#     data = robot.api.get_data(robot.name)
#     if data is None:
#         return

#     state = rmf_easy.RobotState(data.map, data.position, data.battery_soc)

#     if robot.update_handle is None:
#         robot.update_handle = robot.fleet_handle.add_robot(
#             robot.name, state, robot.configuration, robot.make_callbacks()
#         )
#         return

#     robot.update(state, data)


def ros_connections(node, robots: dict[str, RobotAdapter], fleet_handle):
    fleet_name = fleet_handle.more().fleet_name

    transient_qos = QoSProfile(
        history=History.KEEP_LAST,
        depth=1,
        reliability=Reliability.RELIABLE,
        durability=Durability.TRANSIENT_LOCAL,
    )

    closed_lanes_pub = node.create_publisher(
        ClosedLanes, "closed_lanes", qos_profile=transient_qos
    )

    closed_lanes = set()

    def lane_request_cb(msg):
        if msg.fleet_name is None or msg.fleet_name != fleet_name:
            return

        fleet_handle.open_lanes(msg.open_lanes)
        fleet_handle.close_lanes(msg.close_lanes)

        for lane_idx in msg.close_lanes:
            closed_lanes.add(lane_idx)

        for lane_idx in msg.open_lanes:
            closed_lanes.remove(lane_idx)

        state_msg = ClosedLanes()
        state_msg.fleet_name = fleet_name
        state_msg.closed_lanes = list(closed_lanes)
        closed_lanes_pub.publish(state_msg)

    def mode_request_cb(msg: ModeRequest):
        if (
            msg.fleet_name is None
            or msg.fleet_name != fleet_name
            or msg.robot_name is None
        ):
            return

        robot = robots.get(msg.robot_name)
        if robot is None:
            return

        if msg.mode.mode == RobotMode.MODE_IDLE:
            # robot = robots.get(msg.robot_name)
            # if robot is None:
            #     return
            robot.finish_action()
        elif msg.mode.mode == RobotMode.MODE_PAUSED:
            robot.pause()
        elif msg.mode.mode == RobotMode.MODE_MOVING:
            robot.resume()

    node.create_subscription(
        LaneRequest,
        "lane_closure_requests",
        lane_request_cb,
        qos_profile=qos_profile_system_default,
    )

    node.create_subscription(
        ModeRequest,
        "action_execution_notice",
        mode_request_cb,
        qos_profile=qos_profile_system_default,
    )


if __name__ == "__main__":
    main(sys.argv)
