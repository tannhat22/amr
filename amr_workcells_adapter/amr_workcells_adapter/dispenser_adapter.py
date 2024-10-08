#!/usr/bin/env python3

import sys
import yaml
import argparse
from uuid import uuid4
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from rmf_dispenser_msgs.msg import DispenserRequest, DispenserResult, DispenserState
from machine_fleet_msgs.msg import (
    FleetMachineState,
    MachineState,
    MachineMode,
    MachineRequest,
    StationMode,
    StationRequest,
)


class State:
    def __init__(self, state: MachineState = None):
        self.state = state
        self.last_completed_request = None


class DispenserHandle(Node):

    def __init__(self, config, nav_graph):
        self.config = config
        self.nav_graph = nav_graph

        super().__init__("dispenser_manager")

        sub_cb_group = ReentrantCallbackGroup()
        # timer_cb_group = MutuallyExclusiveCallbackGroup()

        # Publisher, Subcribers:
        # self.dispenser_state_pub = self.create_publisher(DispenserState, "/dispenser_states", 10)
        self.dispenser_result_pub = self.create_publisher(
            DispenserResult, "/dispenser_results", 10
        )
        self.machineRequestPub = self.create_publisher(
            MachineRequest, "/machine_request", 10
        )
        self.stationRequestPub = self.create_publisher(
            StationRequest, "/station_request", 10
        )

        self.dispenser_request_sub = self.create_subscription(
            DispenserRequest,
            "/dispenser_requests",
            self.handle_request_cb,
            1,
            callback_group=sub_cb_group,
        )
        self.machines_state_sub = self.create_subscription(
            FleetMachineState,
            "fleet_machine_state",
            self.machine_states_cb,
            100,
            callback_group=sub_cb_group,
        )

        # Variables:
        self.fleet_name = self.config["rmf_fleet"]["name"]
        self.stations = self.config["stations"]
        self.machines = {}
        self.request_guid = None

        for machine_name in self.config["machines"]:
            self.machines[machine_name] = State()
        # assert len(self.machines) > 0

        # timer_period = 0.5
        # self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=timer_cb_group)
        # self.get_logger().info('Beginning dispenser_manager node, shut down with CTRL-C')
        # self.get_logger().info(f'Fleet name: {self.fleet_name}')

    def handle_request_cb(self, msg: DispenserRequest):
        if self.request_guid != msg.request_guid:
            self.request_guid = msg.request_guid
            self.get_logger().info(
                f"Receive dispenser_requests: {msg.request_guid}, "
                f"will publish success for this request"
            )
            name = msg.target_guid
            if name in self.machines:
                msgMR = MachineRequest()
                msgMR.fleet_name = self.fleet_name
                msgMR.machine_name = name
                msgMR.request_id = str(uuid4())[0:8]
                msgMR.mode.mode = MachineMode.MODE_PK_RELEASE
                self.machineRequestPub.publish(msgMR)
                while (
                    not self.machines[name].last_completed_request == msgMR.request_id
                ):
                    self.get_logger().info(f"Waiting machine completed request!")
                    self.get_logger().info(f"Request ID: {msgMR.request_id}")
                    self.get_logger().info(
                        f"Machine-[{name}]--last_completed_request: {self.machines[name].last_completed_request}"
                    )
                    time.sleep(1.0)
                    continue

            elif name in self.stations:
                msgSR = StationRequest()
                msgSR.fleet_name = self.fleet_name
                msgSR.machine_name = "nqvlm104"
                msgSR.station_name = name
                msgSR.request_id = str(uuid4())[0:8]
                msgSR.mode.mode = StationMode.MODE_EMPTY
                self.stationRequestPub.publish(msgSR)

            else:
                self.get_logger().warn(
                    f"Dispenser name not found in machines or stations, return DispenserResult.SUCCESS!"
                )

            result_msg = DispenserResult()
            t = self.get_clock().now().to_msg()
            result_msg.time = t
            result_msg.request_guid = msg.request_guid
            result_msg.source_guid = msg.target_guid
            result_msg.status = DispenserResult.SUCCESS
            self.dispenser_result_pub.publish(result_msg)
        return

    def machine_states_cb(self, msg: FleetMachineState):
        if msg.name == self.fleet_name:
            dataMachine = msg.machines
            for machineMsg in dataMachine:
                if machineMsg.machine_name in self.machines.keys():
                    machine = self.machines[machineMsg.machine_name]
                    machine.state = machineMsg

                    if machineMsg.mode.mode == MachineMode.MODE_IDLE:
                        completed_request = machineMsg.request_id
                        if machine.last_completed_request != completed_request:
                            # if self.debug:
                            #     print(
                            #         f'Detecting completed request for {robotMsg.name}: '
                            #         f'{completed_request}'
                            #     )
                            machine.last_completed_request = completed_request
                else:
                    self.get_logger().warn(
                        f'Detect machine "{machineMsg.machine_name}" is not in config file, pleascheck!'
                    )


def main(argv=sys.argv):
    # Init rclpy and adapter
    rclpy.init(args=argv)
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="dispenser_adapter",
        description="Configure and spin up the dispenser adapter",
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
        help="Path to the nav_graph for this dispenser adapter",
    )
    args = parser.parse_args(args_without_ros[1:])

    with open(args.config_file, "r") as f:
        config = yaml.safe_load(f)

    dispenser_handle = DispenserHandle(config, args.nav_graph)
    executor = MultiThreadedExecutor()
    executor.add_node(dispenser_handle)

    try:
        dispenser_handle.get_logger().info(
            "Beginning dispenser_manager node, shut down with CTRL-C"
        )
        executor.spin()
    except KeyboardInterrupt:
        dispenser_handle.get_logger().info("Keyboard interrupt, shutting down.\n")
    dispenser_handle.destroy_node()
    rclpy.shutdown()
    # rclpy.spin(dispenser_handle)
    # dispenser_handle.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    main()
