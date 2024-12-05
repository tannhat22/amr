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
import sys
import threading
import time
import re

import rclpy
from rclpy.duration import Duration
import rclpy.node
import rclpy.publisher
from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy as Reliability

from rmf_dispenser_msgs.msg import DispenserRequest, DispenserResult, DispenserState
from rmf_ingestor_msgs.msg import IngestorRequest, IngestorResult, IngestorState
from machine_fleet_msgs.msg import DeviceMode, MachineRequest
import yaml

from .MachineClientAPI import MachineAPI
from .MachineClientAPI import MachineAPIResult
from .MachineClientAPI import MachineUpdateData


def search_mode_docking(dock_name: str):
    match = re.search(r"--(.+)", dock_name)
    if match:
        result = match.group(1)
        return result
    else:
        return None


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def main(argv=sys.argv):
    # Init rclpy and adapter
    rclpy.init(args=argv)
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="workcell_adapter",
        description="Configure and spin up the workcell adapter",
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
        help="Path to the nav_graph for this workcell adapter",
    )

    args = parser.parse_args(args_without_ros[1:])
    print("Starting workcell adapter...")

    config_path = args.config_file
    nav_graph_path = args.nav_graph

    # Parse the yaml in Python to get the workcell_manager info
    with open(config_path, "r") as f:
        config_yaml = yaml.safe_load(f)

    with open(nav_graph_path, "r") as f:
        nav_graph = yaml.safe_load(f)

    # ROS 2 node for the command handle
    node = rclpy.node.Node("workcell_command_handle")

    time.sleep(1.0)

    # Initialize robot API for this workcell
    workcell_mgr_yaml = config_yaml["workcell_manager"]
    update_period = 1.0 / workcell_mgr_yaml.get("workcell_state_update_frequency", 10.0)
    api_prefix = "http://" + workcell_mgr_yaml["ip"] + ":" + str(workcell_mgr_yaml["port"])
    api = MachineAPI(api_prefix, workcell_mgr_yaml["user"], workcell_mgr_yaml["password"])

    # Publishers:
    dispenser_state_pub = node.create_publisher(DispenserState, "/dispenser_states", 10)

    dispenser_result_pub = node.create_publisher(DispenserResult, "/dispenser_results", 10)

    ingestor_state_pub = node.create_publisher(IngestorState, "/ingestor_states", 10)

    ingestor_result_pub = node.create_publisher(IngestorResult, "/ingestor_results", 10)

    dispensers: dict[str, DispenserAdapter]
    dispensers = {}

    ingestors: dict[str, IngestorAdapter]
    ingestors = {}

    for level in nav_graph["levels"]:
        for wp in nav_graph["levels"][level]["vertices"]:
            assert len(wp) == 3, "Vertical structure not match, please check!"

            # add Dispenser context
            if "pickup_dispenser" in wp[2]:
                dispenser_name = wp[2]["pickup_dispenser"]
                assert (
                    dispenser_name not in dispensers
                ), f"Pickup_dispenser [{dispenser_name}] is duplicated, please check!"

                is_machine = False
                if search_mode_docking(wp[2]["dock_name"]) == "mpickup":
                    is_machine = True

                dispensers[dispenser_name] = DispenserAdapter(
                    name=dispenser_name,
                    node=node,
                    state_pub=dispenser_state_pub,
                    result_pub=dispenser_result_pub,
                    is_machine=is_machine,
                    api=api,
                )

            # add Ingestor context
            elif "dropoff_ingestor" in wp[2]:
                ingestor_name = wp[2]["dropoff_ingestor"]
                assert (
                    ingestor_name not in ingestors
                ), f"Dropoff_ingestor [{ingestor_name}] is duplicated, please check!"

                is_machine = False
                if search_mode_docking(wp[2]["dock_name"]) == "mdropoff":
                    is_machine = True

                ingestors[ingestor_name] = IngestorAdapter(
                    name=ingestor_name,
                    node=node,
                    state_pub=ingestor_state_pub,
                    result_pub=ingestor_result_pub,
                    is_machine=is_machine,
                    api=api,
                )

    async def state_updates():
        workcell_updaters = []
        for dispenser in dispensers.values():
            workcell_updaters.append(dispenser.update_loop(update_period))

        for ingestor in ingestors.values():
            workcell_updaters.append(ingestor.update_loop(update_period))

        await asyncio.gather(*workcell_updaters)

    def update_loop():
        event_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(event_loop)
        event_loop.run_until_complete(state_updates())

    update_thread = threading.Thread(target=update_loop, args=())
    update_thread.start()

    # Connect to the extra ROS2 topics that are relevant for the adapter
    # ros_connections(node, machines, stations)

    transient_qos = QoSProfile(
        history=History.KEEP_LAST,
        depth=1,
        reliability=Reliability.RELIABLE,
        durability=Durability.TRANSIENT_LOCAL,
    )

    # Handle dispenser request
    def dispenser_request_cb(msg: DispenserRequest):
        if not msg.target_guid:
            return

        dispenser = dispensers.get(msg.target_guid, None)
        if dispenser is not None:
            dispenser.dispenserRequest(msg.request_guid)

    # Handle ingestor request
    def ingestor_request_cb(msg: IngestorRequest):
        if not msg.target_guid:
            return

        ingestor = ingestors.get(msg.target_guid, None)
        if ingestor is not None:
            ingestor.ingestorRequest(msg.request_guid)

    def machine_request_cb(msg: MachineRequest):
        if msg.request_type == MachineRequest.REQUEST_DISPENSER:
            dispenser = dispensers.get(msg.machine_name, None)
            if dispenser is not None:
                dispenser.machineRequest(msg.request_mode.mode)
        elif msg.request_type == MachineRequest.REQUEST_INGESTOR:
            ingestor = ingestors.get(msg.machine_name, None)
            if ingestor is not None:
                ingestor.machineRequest(msg.request_mode.mode)

    node.create_subscription(
        DispenserRequest,
        "/dispenser_requests",
        dispenser_request_cb,
        qos_profile=qos_profile_system_default,
    )

    node.create_subscription(
        IngestorRequest,
        "/ingestor_requests",
        ingestor_request_cb,
        qos_profile=qos_profile_system_default,
    )

    node.create_subscription(
        MachineRequest,
        "/adapter_machine_requests",
        machine_request_cb,
        qos_profile=qos_profile_system_default,
    )

    # Create executor for the command handle node
    rclpy_executor = rclpy.executors.SingleThreadedExecutor()
    rclpy_executor.add_node(node)

    # Start the fleet adapter
    rclpy_executor.spin()

    # Shutdown
    node.destroy_node()
    rclpy_executor.shutdown()
    rclpy.shutdown()


# Parallel processing solution derived from
# https://stackoverflow.com/a/59385935
def parallel(f):
    def run_in_parallel(*args, **kwargs):
        return asyncio.get_event_loop().run_in_executor(None, f, *args, **kwargs)

    return run_in_parallel


class DispenserAdapter:
    def __init__(
        self,
        name: str,
        node,
        state_pub: rclpy.publisher.Publisher,
        result_pub: rclpy.publisher.Publisher,
        is_machine: bool = False,
        api: MachineAPI | None = None,
    ):
        self.name = name
        self.cmd_id = 0
        self.node = node
        self.state_pub = state_pub
        self.result_pub = result_pub
        self.api = api
        self.is_machine = is_machine
        self.request_guid = None
        self.past_request_guids = []
        self.disconnect = False
        self.state = DispenserState()
        self.state.guid = name
        self.state.mode = DispenserState.IDLE

        if is_machine:
            assert api is not None, f"Dispenser [{name}] is machine but no configure api!"

        # Threading variables
        self._lock = threading.Lock()
        self.issue_cmd_thread = None
        self.cancel_cmd_event = threading.Event()

    async def update_loop(self, period):
        while rclpy.ok():
            now = self.node.get_clock().now()
            next_wakeup = now + Duration(nanoseconds=period * 1e9)
            if self.is_machine:
                data = self.api.get_data(self.name)
                if data is None:
                    if not self.disconnect:
                        self.disconnect = True
                        self.node.get_logger().warn(
                            f"Unable to retrieve state from machine [{self.name}]!"
                        )

                    self.update_dispenser_state(None)
                else:
                    if self.disconnect:
                        self.node.get_logger().info(
                            f"Machine [{self.name}] connectivity resumed, received "
                            f"status from machine successfully."
                        )
                        self.disconnect = False

                    await self.update(data)
            else:
                await self.update(None)

            while self.node.get_clock().now() < next_wakeup:
                await asyncio.sleep(0.01)

    @parallel
    def update(self, data: MachineUpdateData | None):
        with self._lock:
            request_guid = self.request_guid

            # Update state:
            self.update_dispenser_state(request_guid)

            if request_guid is None:
                return

            self.update_dispenser_result(data, request_guid)

    def update_dispenser_state(self, request_guid):
        if self.disconnect:
            self.state.mode = DispenserState.OFFLINE
        elif request_guid is None:
            self.state.mode = DispenserState.IDLE
            self.state.request_guid_queue.clear()
        else:
            self.state.mode = DispenserState.BUSY
            if request_guid not in self.state.request_guid_queue:
                self.state.request_guid_queue.append(request_guid)

        self.state.time = self.node.get_clock().now().to_msg()
        self.state_pub.publish(self.state)
        return

    def update_dispenser_result(self, status: MachineUpdateData, request_guid: str):
        msgResult = DispenserResult()
        msgResult.time = self.node.get_clock().now().to_msg()
        msgResult.request_guid = request_guid
        msgResult.source_guid = self.name
        msgResult.status = DispenserResult.SUCCESS

        if self.is_machine and not status.is_command_dispenser_completed(self.cmd_id):
            return

        self.result_pub.publish(msgResult)
        self.past_request_guids.append(request_guid)
        memory_leak = len(self.past_request_guids)
        if memory_leak > 5:
            self.past_request_guids = self.past_request_guids[memory_leak - 5 :]
        self.request_guid = None

    def dispenserRequest(self, request_guid: str):
        with self._lock:
            if self.request_guid is not None:
                return

            # This request is handle in past
            if request_guid in self.past_request_guids:
                return

            self.node.get_logger().info(
                f"Dispenser request_guid: [{request_guid}] command [{self.name}] to process, cmd_id {self.cmd_id}."
            )
            msgResult = DispenserResult()
            msgResult.time = self.node.get_clock().now().to_msg()
            msgResult.request_guid = request_guid
            msgResult.source_guid = self.name
            msgResult.status = DispenserResult.ACKNOWLEDGED
            self.result_pub.publish(msgResult)
            self.request_guid = request_guid

            if self.is_machine:
                self.cmd_id += 1
                self.attempt_cmd_until_success(
                    cmd=self.perform_operate,
                    args=(DeviceMode.MODE_ACCEPT_DOCKOUT,),
                )

    def machineRequest(self, mode: int):
        with self._lock:
            self.cmd_id += 1
            self.attempt_cmd_until_success(
                cmd=self.perform_operate,
                args=(mode,),
            )

    def stop(self):
        with self._lock:
            request_guid = self.request_guid
            if request_guid is not None:
                self.node.get_logger().info(f"Dispenser [{self.name}] stop requested from RMF!")
                self.request_guid = None

    def perform_operate(self, mode: int):
        match self.api.start_activity(self.name, self.cmd_id, "dispenser", mode):
            case MachineAPIResult.SUCCESS:
                return True
            case MachineAPIResult.IMPOSSIBLE:
                return False
            case MachineAPIResult.RETRY:
                return False

    def attempt_cmd_until_success(self, cmd, args):
        self.cancel_cmd_attempt()

        def loop():
            while not cmd(*args):
                self.node.get_logger().warn(
                    f"Failed to contact workcell manager for machine {self.name}"
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


class IngestorAdapter:
    def __init__(
        self,
        name: str,
        node,
        state_pub: rclpy.publisher.Publisher,
        result_pub: rclpy.publisher.Publisher,
        is_machine: bool = False,
        api: MachineAPI | None = None,
    ):
        self.name = name
        self.cmd_id = 0
        self.node = node
        self.state_pub = state_pub
        self.result_pub = result_pub
        self.api = api
        self.is_machine = is_machine
        self.request_guid = None
        self.past_request_guids = []
        self.disconnect = False
        self.state = IngestorState()
        self.state.guid = name
        self.state.mode = IngestorState.IDLE

        if is_machine:
            assert api is not None, f"Ingestor [{name}] is machine but no configure api!"

        # Threading variables
        self._lock = threading.Lock()
        self.issue_cmd_thread = None
        self.cancel_cmd_event = threading.Event()

    async def update_loop(self, period):
        while rclpy.ok():
            now = self.node.get_clock().now()
            next_wakeup = now + Duration(nanoseconds=period * 1e9)
            if self.is_machine:
                data = self.api.get_data(self.name)
                if data is None:
                    if not self.disconnect:
                        self.disconnect = True
                        self.node.get_logger().warn(
                            f"Unable to retrieve state from machine [{self.name}]!"
                        )
                    self.update_ingestor_state(None)
                else:
                    if self.disconnect:
                        self.node.get_logger().info(
                            f"Machine [{self.name}] connectivity resumed, received "
                            f"status from machine successfully."
                        )
                        self.disconnect = False

                    await self.update(data)
            else:
                await self.update(None)

            while self.node.get_clock().now() < next_wakeup:
                await asyncio.sleep(0.01)

    @parallel
    def update(self, data: MachineUpdateData | None):
        with self._lock:
            request_guid = self.request_guid

            # Update state:
            self.update_ingestor_state(request_guid)

            if request_guid is None:
                return

            self.update_ingestor_result(data, request_guid)

    def update_ingestor_state(self, request_guid):
        if self.disconnect:
            self.state.mode = IngestorState.OFFLINE
        elif request_guid is None:
            self.state.mode = IngestorState.IDLE
            self.state.request_guid_queue.clear()
        else:
            self.state.mode = IngestorState.BUSY
            if request_guid not in self.state.request_guid_queue:
                self.state.request_guid_queue.append(request_guid)

        self.state.time = self.node.get_clock().now().to_msg()
        self.state_pub.publish(self.state)
        return

    def update_ingestor_result(self, status: MachineUpdateData, request_guid: str):
        msgResult = IngestorResult()
        msgResult.time = self.node.get_clock().now().to_msg()
        msgResult.request_guid = request_guid
        msgResult.source_guid = self.name
        msgResult.status = IngestorResult.SUCCESS

        if self.is_machine and not status.is_command_ingestor_completed(self.cmd_id):
            return

        self.result_pub.publish(msgResult)
        self.past_request_guids.append(request_guid)
        memory_leak = len(self.past_request_guids)
        if memory_leak > 5:
            self.past_request_guids = self.past_request_guids[memory_leak - 5 :]
        self.request_guid = None

    def ingestorRequest(self, request_guid: str):
        with self._lock:
            if self.request_guid is not None:
                return

            # This request is handle in past
            if request_guid in self.past_request_guids:
                return

            self.node.get_logger().info(
                f"Ingestor request_guid: [{request_guid}] command [{self.name}] to process, cmd_id {self.cmd_id}."
            )
            msgResult = IngestorResult()
            msgResult.time = self.node.get_clock().now().to_msg()
            msgResult.request_guid = request_guid
            msgResult.source_guid = self.name
            msgResult.status = IngestorResult.ACKNOWLEDGED
            self.result_pub.publish(msgResult)
            self.request_guid = request_guid

            if self.is_machine:
                self.cmd_id += 1
                self.attempt_cmd_until_success(
                    cmd=self.perform_operate,
                    args=(DeviceMode.MODE_ACCEPT_DOCKOUT,),
                )

    def machineRequest(self, mode: int):
        with self._lock:
            self.cmd_id += 1
            self.attempt_cmd_until_success(
                cmd=self.perform_operate,
                args=(mode,),
            )

    def stop(self):
        with self._lock:
            request_guid = self.request_guid
            if request_guid is not None:
                self.node.get_logger().info(f"Ingestor [{self.name}] stop requested from RMF!")
                self.request_guid = None

    def perform_operate(self, mode: int):
        match self.api.start_activity(self.name, self.cmd_id, "ingestor", mode):
            case MachineAPIResult.SUCCESS:
                return True
            case MachineAPIResult.IMPOSSIBLE:
                return False
            case MachineAPIResult.RETRY:
                return False

    def attempt_cmd_until_success(self, cmd, args):
        self.cancel_cmd_attempt()

        def loop():
            while not cmd(*args):
                self.node.get_logger().warn(
                    f"Failed to contact workcell manager for machine {self.name}"
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


if __name__ == "__main__":
    main(sys.argv)
