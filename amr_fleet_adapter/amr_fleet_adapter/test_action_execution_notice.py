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
import argparse

import rclpy
import rclpy.qos
from rclpy.qos import qos_profile_system_default
from rclpy.node import Node

from rmf_fleet_msgs.msg import ModeRequest, RobotMode


def main(argv=sys.argv):
    """
    Example lift request:
    - fleet_name: amr_vdm
    - robot_name: amr001
    - request_mode: pause
    """

    default_fleet_name = "amr_tayrua"
    default_robot_name = "amr001"
    default_request_mode = "pause"
    default_topic_name = "/action_execution_notice"

    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--fleet-name", default=default_fleet_name)
    parser.add_argument("-r", "--robot-name", default=default_robot_name)
    parser.add_argument("-m", "--request-mode", default=default_request_mode)
    parser.add_argument("-t", "--topic-name", default=default_topic_name)
    args = parser.parse_args(argv[1:])

    print("fleet_name: {}".format(args.fleet_name))
    print("robot_name: {}".format(args.robot_name))
    print("request_mode: {}".format(args.request_mode))
    print("topic_name: {}".format(args.topic_name))

    transient_qos = rclpy.qos.QoSProfile(
        history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
    )

    rclpy.init()
    node = rclpy.create_node("send_action_execution_notice_node")
    pub = node.create_publisher(
        ModeRequest, args.topic_name, qos_profile=qos_profile_system_default
    )

    msg = ModeRequest()
    msg.fleet_name = args.fleet_name
    msg.robot_name = args.robot_name

    if args.request_mode == "mode":
        print("Please insert desired request_mode: pause or resume")
        return
    elif args.request_mode == "pause":
        msg.mode.mode = RobotMode.MODE_PAUSED
    elif args.request_mode == "resume":
        msg.mode.mode = RobotMode.MODE_MOVING
    else:
        print("unrecognized request_mode, only use pause or resume please")
        return

    rclpy.spin_once(node, timeout_sec=1.0)
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.5)
    print("all done!")
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
