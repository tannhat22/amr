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


# import asyncio
import json
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rmf_task_msgs.msg import ApiRequest

# from rmf_task_msgs.msg import ApiResponse
from rmf_task_msgs.msg import TaskRequest

###############################################################################


class TaskRequester(Node):

    def __init__(self):
        super().__init__("amr_delivery_requester")

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL,
        )

        # Publishers:
        self.task_api_req_pub = self.create_publisher(
            ApiRequest, "task_api_requests", transient_qos
        )

        # Subcribers:
        # self.sub = self.create_subscription(
        #     ApiResponse, "task_api_responses", self.receive_response, 10
        # )

        self.create_subscription(
            TaskRequest, "amr_delivery_requests", self.task_requests_callback, 10
        )

        self.get_logger().info(
            "Beginning amr_delivery_requester node, shut down with CTRL-C"
        )

    def task_requests_callback(self, msg: TaskRequest):
        pickups = json.loads(msg.pickups)
        dropoffs = json.loads(msg.dropoffs)
        self.dispatch_delivery(
            msg.fleet_name,
            msg.robot_name,
            msg.start_time,
            msg.requester,
            pickups,
            dropoffs,
        )

    # pickup: {place: name_pickup_waypoint, handler: pickup_dispenser, sku_qty: ["sku", "quantity"]}
    def __create_pickup_desc(self, pickup: dict):
        place = pickup["place"]
        handler = pickup["handler"]
        sku_qty = pickup["sku_qty"]
        assert len(sku_qty) == 2, "please specify sku and qty for pickup payload"
        payload = [{"sku": sku_qty[0], "quantity": int(sku_qty[1])}]

        return {
            "place": place,
            "handler": handler,
            "payload": payload,
        }

    # dropoff: {place: name_dropoff_waypoint, handler: dropoff_ingestor, sku_qty: ["sku", "quantity"]}
    def __create_dropoff_desc(self, dropoff: dict):
        place = dropoff["place"]
        handler = dropoff["handler"]
        sku_qty = dropoff["sku_qty"]
        assert len(sku_qty) == 2, "please specify sku and qty for dropoff payload"
        payload = [{"sku": sku_qty[0], "quantity": int(sku_qty[1])}]

        return {
            "place": place,
            "handler": handler,
            "payload": payload,
        }

    def dispatch_delivery(
        self,
        fleet: str = None,
        robot: str = None,
        start_time_task: int = 0,
        requester: str = "amr_task",
        pickups: list = [],
        dropoffs: list = [],
    ):
        assert (
            len(pickups) > 0 and len(dropoffs) > 0 and len(pickups) == len(dropoffs)
        ), "pickups and dropoffs invalid, please check!"

        # Construct task
        msg = ApiRequest()
        msg.request_id = "delivery_" + str(uuid.uuid4())
        payload = {}
        if fleet and robot:
            self.get_logger().info("Using 'robot_task_request'")
            payload["type"] = "robot_task_request"
            payload["fleet"] = fleet
            payload["robot"] = robot
        else:
            self.get_logger().info("Using 'dispatch_task_request'")
            payload["type"] = "dispatch_task_request"
        request = {}

        # Set task request request time, start time and requester
        now = self.get_clock().now().to_msg()
        now.sec = now.sec + start_time_task
        start_time = now.sec * 1000 + round(now.nanosec / 10**6)
        request["unix_millis_request_time"] = start_time
        request["unix_millis_earliest_start_time"] = start_time
        request["requester"] = requester

        if fleet:
            request["fleet_name"] = fleet

        # Use standard delivery task type
        if len(pickups) == 1 and len(dropoffs) == 1:
            request["category"] = "delivery"
            description = {
                "pickup": self.__create_pickup_desc(pickups[0]),
                "dropoff": self.__create_dropoff_desc(dropoffs[0]),
            }
        else:
            # Define multi_delivery with request category compose
            request["category"] = "compose"

            # Define task request description with phases
            description = {}  # task_description_Compose.json
            description["category"] = "multi_delivery"
            description["phases"] = []
            activities = []
            for i in range(0, len(pickups)):
                # Add each pickup
                activities.append(
                    {
                        "category": "pickup",
                        "description": self.__create_pickup_desc(pickups[i]),
                    }
                )
                # Add each dropoff
                activities.append(
                    {
                        "category": "dropoff",
                        "description": self.__create_dropoff_desc(dropoffs[i]),
                    }
                )

            # Add activities to phases
            description["phases"].append(
                {
                    "activity": {
                        "category": "sequence",
                        "description": {"activities": activities},
                    }
                }
            )

        request["description"] = description
        payload["request"] = request
        msg.json_msg = json.dumps(payload)

        print(f"Json msg payload: \n{json.dumps(payload, indent=2)}")
        self.task_api_req_pub.publish(msg)

    # def receive_response(self, response_msg: ApiResponse):
    #     if response_msg.request_id == msg.request_id:
    #         self.response.set_result(json.loads(response_msg.json_msg))


###############################################################################


def main(args=None):
    rclpy.init(args=args)

    task_requester = TaskRequester()
    task_requester.dispatch_delivery(
        pickups=[
            {"place": "wp1", "handler": "machine1", "sku_qty": ["FF180", 1]},
            {"place": "wp2", "handler": "machine2", "sku_qty": ["FF180", 1]},
        ],
        dropoffs=[
            {"place": "wp3", "handler": "machine3", "sku_qty": ["FF180", 1]},
            {"place": "wp4", "handler": "machine4", "sku_qty": ["FF180", 1]},
        ],
    )
    rclpy.spin(task_requester)
    task_requester.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
