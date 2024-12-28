#!/usr/bin/env python3

# Copyright 2019 Open Source Robotics Foundation, Inc.
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
import time
import argparse

import rclpy
import rclpy.time
import rclpy.qos
from rclpy.qos import qos_profile_system_default
from rclpy.node import Node

from machine_fleet_msgs.msg import StationRequest
from rclpy.utilities import get_rmw_implementation_identifier


def main(argv=sys.argv):
    """
    Example lift request:
    - station_name: transit90_str--pickup
    - station_type: pickup
    - request_mode: empty
    """

    default_station_name = "transit90_str--pickup"
    default_station_type = "pickup"
    default_request_mode = "empty"
    default_topic_name = "/station_requests"

    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--station-name", default=default_station_name)
    parser.add_argument("-s", "--station-type", default=default_station_type)
    parser.add_argument("-m", "--request-mode", default=default_request_mode)
    parser.add_argument("-t", "--topic-name", default=default_topic_name)
    args = parser.parse_args(argv[1:])

    print("station_name: {}".format(args.station_name))
    print("station_type: {}".format(args.station_type))
    print("request_mode: {}".format(args.request_mode))
    print("topic_name: {}".format(args.topic_name))

    rclpy.init()
    node = rclpy.create_node("send_action_execution_notice_node")
    pub = node.create_publisher(
        StationRequest, args.topic_name, qos_profile=qos_profile_system_default
    )

    msg = StationRequest()
    msg.station_name = args.station_name

    if args.station_type == "type":
        print("Please insert desired station_type: pickup or dropoff")
        return
    elif args.station_type == "pickup":
        msg.station_type = StationRequest.TYPE_PICKUP
    elif args.station_type == "dropoff":
        msg.station_type = StationRequest.TYPE_DROPOFF
    else:
        print("unrecognized station_type, only use empty or filled please")
        return

    if args.request_mode == "mode":
        print("Please insert desired request_mode: empty or filled")
        return
    elif args.request_mode == "empty":
        msg.mode = StationRequest.MODE_EMPTY
    elif args.request_mode == "filled":
        msg.mode = StationRequest.MODE_FILLED
    else:
        print("unrecognized request_mode, only use empty or filled please")
        return

    # msg.time = node.get_clock().now().to_msg()

    rclpy.spin_once(node, timeout_sec=1.0)
    while rclpy.ok():
        msg.time = node.get_clock().now().to_msg()
        pub.publish(msg)
        time.sleep(1.0)
    rclpy.spin_once(node, timeout_sec=0.5)
    print("all done!")
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
