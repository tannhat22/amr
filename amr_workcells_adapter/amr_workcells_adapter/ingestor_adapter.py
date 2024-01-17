#!/usr/bin/env python3

import sys
import yaml
import argparse
import rclpy
from rclpy.node import Node
# from rclpy.executors import MultiThreadedExecutor
# from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from rmf_ingestor_msgs.msg import IngestorRequest, IngestorResult, IngestorState

class IngestorHandle(Node):

    def __init__(self, config, nav_graph):
        self.config = config
        self.nav_graph = nav_graph

        super().__init__("ingestor_manager")

        # sub_cb_group = MutuallyExclusiveCallbackGroup()
        # timer_cb_group = MutuallyExclusiveCallbackGroup()

        # Publisher, Subcribers:
        # self.ingestor_state_pub = self.create_publisher(IngestorState, "/ingestor_states", 10)
        self.ingestor_result_pub = self.create_publisher(IngestorResult, "/ingestor_results", 1)


        # self.ingestor_request_sub = self.create_subscription(IngestorRequest, "/ingestor_requests",
        #                                                       self.handle_request_cb, 10,
        #                                                       callback_group=sub_cb_group)
        self.ingestor_request_sub = self.create_subscription(IngestorRequest, "/ingestor_requests",
                                                              self.handle_request_cb, 10)
        
        # Variables:
        self.fleet_name = self.config["rmf_fleet"]["name"]
        self.dropoff_ingestors = ""

        # timer_period = 0.5
        # self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=timer_cb_group)
        self.get_logger().info('Beginning ingestor_manager node, shut down with CTRL-C')
        # self.get_logger().info(f'Fleet name: {self.fleet_name}')
        

    def handle_request_cb(self, msg: IngestorRequest):
        self.get_logger().info(f'Receive ingestor_requests: {msg.request_guid}, '
                               f'will publish success for this request')
        result_msg = IngestorResult()
        t = self.get_clock().now().to_msg()
        result_msg.time = t
        result_msg.request_guid = msg.request_guid
        result_msg.source_guid = msg.target_guid
        result_msg.status = IngestorResult.SUCCESS
        self.ingestor_result_pub.publish(result_msg)
        return

    # def timer_callback(self):
    #     return


def main(argv=sys.argv):
    # Init rclpy and adapter
    rclpy.init(args=argv)
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="ingestor_adapter",
        description="Configure and spin up the ingestor adapter")
    parser.add_argument("-c", "--config_file", type=str, required=True,
                        help="Path to the config.yaml file")
    parser.add_argument("-n", "--nav_graph", type=str, required=True,
                        help="Path to the nav_graph for this ingestor adapter")
    args = parser.parse_args(args_without_ros[1:])

    with open(args.config_file, "r") as f:
        config = yaml.safe_load(f)

    ingestor_handle = IngestorHandle(config, args.nav_graph)
    # executor = MultiThreadedExecutor()
    # executor.add_node(ingestor_handle)

    # try:
    #     ingestor_handle.get_logger().info('Beginning ingestor_manager node, shut down with CTRL-C')
    #     executor.spin()
    # except KeyboardInterrupt:
    #     ingestor_handle.get_logger().info('Keyboard interrupt, shutting down.\n')
    # ingestor_handle.destroy_node()
    # rclpy.shutdown()
    rclpy.spin(ingestor_handle)
    ingestor_handle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
