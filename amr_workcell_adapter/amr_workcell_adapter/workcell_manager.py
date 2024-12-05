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
import sys
import threading
from typing import Optional

from fastapi import FastAPI
from pydantic import BaseModel
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy as Reliability
from machine_fleet_msgs.msg import (
    DeviceMode,
    FleetMachineState,
    MachineState,
    MachineRequest,
)
import uvicorn
import yaml
import re


app = FastAPI()


class Request(BaseModel):
    activity: Optional[str] = None
    mode: Optional[int] = None


class Response(BaseModel):
    data: Optional[dict] = None
    success: bool
    msg: str


# ------------------------------------------------------------------------------
# Workcell Manager
# ------------------------------------------------------------------------------
class State:
    last_dispenser_request: MachineRequest
    last_ingestor_request: MachineRequest

    def __init__(self, state: MachineState = None):
        self.state = state
        self.last_dispenser_mode_req = None
        self.last_dispenser_request = None
        self.last_dispenser_completed_request = None
        self.last_ingestor_mode_req = None
        self.last_ingestor_request = None
        self.last_ingestor_completed_request = None

    def is_expected_dispenser_task_id(self, task_id):
        if self.last_dispenser_request is not None:
            if task_id != self.last_dispenser_request.request_id:
                return False
        return True

    def is_expected_ingestor_task_id(self, task_id):
        if self.last_ingestor_request is not None:
            if task_id != self.last_ingestor_request.request_id:
                return False
        return True


def search_mode_docking(dock_name: str):
    match = re.search(r"--(.+)", dock_name)
    if match:
        result = match.group(1)
        return result
    else:
        return None


class WorkCellManager(Node):
    machines: dict[str, State]

    def __init__(self, config, nav_graph):
        self.debug = False
        self.config = config

        super().__init__("workcell_manager")

        self.machines = {}  # Map machine name to state

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
                    self.machines[machine_name] = State()

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

        self.machine_req_pub = self.create_publisher(
            MachineRequest,
            "machine_requests",
            qos_profile=qos_profile_system_default,
        )

        @app.get("/open-rmf/rmf_vdm_wm/status/", response_model=Response)
        async def status(machine_name: Optional[str] = None):
            response = {"data": {}, "success": False, "msg": ""}
            if machine_name is None:
                response["data"]["all_machines"] = []
                for machine_name in self.machines:
                    state = self.machines.get(machine_name)
                    if state is None or state.state is None:
                        return response
                    response["data"]["all_machines"].append(
                        self.get_machine_state(state, machine_name)
                    )
            else:
                state = self.machines.get(machine_name)
                if state is None or state.state is None:
                    return response
                response["data"] = self.get_machine_state(state, machine_name)
            response["success"] = True
            return response

        @app.post("/open-rmf/rmf_vdm_wm/start_activity/", response_model=Response)
        async def start_activity(machine_name: str, cmd_id: int, request: Request):
            response = {"success": False, "msg": ""}
            if machine_name not in self.machines:
                return response

            machine = self.machines[machine_name]
            machine_request = MachineRequest()

            if request.activity == "dispenser":
                machine_request.request_type = MachineRequest.REQUEST_DISPENSER
            elif request.activity == "ingestor":
                machine_request.request_type = MachineRequest.REQUEST_INGESTOR
            else:
                response["msg"] = "Request type does not support. Please check activity!"
                return response

            if request.mode == DeviceMode.MODE_IDLE:
                machine_request.request_mode.mode = DeviceMode.MODE_IDLE
            elif request.mode == DeviceMode.MODE_ACCEPT_DOCKIN:
                machine_request.request_mode.mode = DeviceMode.MODE_ACCEPT_DOCKIN
            elif request.mode == DeviceMode.MODE_ROBOT_DOCKED_IN:
                machine_request.request_mode.mode = DeviceMode.MODE_ROBOT_DOCKED_IN
            elif request.mode == DeviceMode.MODE_ACCEPT_DOCKOUT:
                machine_request.request_mode.mode = DeviceMode.MODE_ACCEPT_DOCKOUT
            elif request.mode == DeviceMode.MODE_CANCEL:
                machine_request.request_mode.mode = DeviceMode.MODE_CANCEL
            elif request.mode == DeviceMode.MODE_ROBOT_ERROR:
                machine_request.request_mode.mode = DeviceMode.MODE_ROBOT_ERROR
            else:
                response["msg"] = "Mode device request does not support. Please check mode!"
                return response

            machine_request.machine_name = machine_name
            machine_request.request_id = str(cmd_id)
            machine_request.time = self.get_clock().now().to_msg()
            self.machine_req_pub.publish(machine_request)

            if self.debug:
                print(
                    f"Sending [{request.activity}] mode [{request.mode}] "
                    f"request for {machine_name}: {cmd_id}"
                )

            if machine_request.request_type == MachineRequest.REQUEST_DISPENSER:
                machine.last_dispenser_request = machine_request
                if (
                    request.mode == DeviceMode.MODE_CANCEL
                    or request.mode == DeviceMode.MODE_ROBOT_ERROR
                ):
                    machine.last_dispenser_mode_req = None
                else:
                    machine.last_dispenser_mode_req = request.mode
            else:
                machine.last_ingestor_request = machine_request
                if (
                    request.mode == DeviceMode.MODE_CANCEL
                    or request.mode == DeviceMode.MODE_ROBOT_ERROR
                ):
                    machine.last_ingestor_mode_req = None
                else:
                    machine.last_ingestor_mode_req = request.mode
            response["success"] = True
            return response

        # ///////////////////////////////////////////////////////////////////////
        # ///////////////////////////////////////////////////////////////////////

    def fleet_machine_state_cb(self, msg: FleetMachineState):
        dataMachine = msg.machines
        machineMsg: MachineState
        for machineMsg in dataMachine:
            if machineMsg.machine_name in self.machines:
                machine = self.machines[machineMsg.machine_name]

                # Check republish for dispenser
                if not machine.is_expected_dispenser_task_id(machineMsg.dispenser_request_id):
                    # This message is out of date, so disregard it.
                    if machine.last_dispenser_request is not None:
                        # Resend the latest dispenser request for this machine, in case
                        # the message was dropped.
                        if not self.debug:
                            print(
                                f"Republishing dispenser request for {machineMsg.machine_name}: "
                                f"{machine.last_dispenser_request.request_id}, "
                                f"because it is currently following {machineMsg.dispenser_request_id}"
                            )
                        if type(machine.last_dispenser_request) is MachineRequest:
                            self.machine_req_pub.publish(machine.last_dispenser_request)
                    continue

                # Check republish for ingestor
                if not machine.is_expected_ingestor_task_id(machineMsg.ingestor_request_id):
                    # This message is out of date, so disregard it.
                    if machine.last_ingestor_request is not None:
                        # Resend the latest ingestor request for this machine, in case
                        # the message was dropped.
                        if not self.debug:
                            print(
                                f"Republishing ingestor request for {machineMsg.machine_name}: "
                                f"{machine.last_ingestor_request.request_id}, "
                                f"because it is currently following {machineMsg.ingestor_request_id}"
                            )
                        if type(machine.last_ingestor_request) is MachineRequest:
                            self.machine_req_pub.publish(machine.last_ingestor_request)
                    continue

                machine.state = machineMsg

                # Check dispenser request is completed
                if machine.last_dispenser_mode_req is not None:
                    if machine.last_dispenser_mode_req == machine.state.dispenser_mode.mode:
                        completed_request = int(machineMsg.dispenser_request_id)
                        if machine.last_dispenser_completed_request != completed_request:
                            if self.debug:
                                print(
                                    f"Detecting completed dispenser request for {machineMsg.machine_name}: "
                                    f"{completed_request}"
                                )
                        machine.last_dispenser_completed_request = completed_request
                        machine.last_dispenser_mode_req = None

                # Check ingestor request is completed
                if machine.last_ingestor_mode_req is not None:
                    if machine.last_ingestor_mode_req == machine.state.dispenser_mode.mode:
                        completed_request = int(machineMsg.ingestor_request_id)
                        if machine.last_ingestor_completed_request != completed_request:
                            if self.debug:
                                print(
                                    f"Detecting completed ingestor request for {machineMsg.machine_name}: "
                                    f"{completed_request}"
                                )
                        machine.last_ingestor_completed_request = completed_request
                        machine.last_ingestor_mode_req = None

            else:
                self.get_logger().warn(
                    f'Detect machine "{machineMsg.machine_name}" is not in config file, pleascheck!'
                )

    def get_machine_state(self, machine: State, machine_name):
        data = {}
        data["machine_name"] = machine_name
        data["last_dispenser_completed_request"] = machine.last_dispenser_completed_request
        data["last_ingestor_completed_request"] = machine.last_ingestor_completed_request
        data["mode"] = machine.state.machine_mode
        return data


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
        help="Path to the nav_graph for this workcell manager",
    )
    args = parser.parse_args(args_without_ros[1:])
    print("Starting workcell manager...")

    config_path = args.config_file
    nav_graph_path = args.nav_graph

    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    with open(nav_graph_path, "r") as f:
        nav_graph = yaml.safe_load(f)

    workcell_manager = WorkCellManager(config, nav_graph)

    spin_thread = threading.Thread(target=rclpy.spin, args=(workcell_manager,))
    spin_thread.start()

    uvicorn.run(
        app,
        host=config["workcell_manager"]["ip"],
        port=config["workcell_manager"]["port"],
        log_level="warning",
    )


if __name__ == "__main__":
    main(sys.argv)
