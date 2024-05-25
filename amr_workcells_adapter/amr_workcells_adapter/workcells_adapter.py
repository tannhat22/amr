#!/usr/bin/env python3

from uuid import uuid4
from datetime import datetime
import json
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSReliabilityPolicy as Reliability

from rmf_task_msgs.msg import ApiRequest, ApiResponse
from machine_fleet_msgs.msg import DeliveryMode, DeliveryRequest

class WorkcellHandle(Node):

    def __init__(self):
        super().__init__("workcells_manager")

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL)

        # Publisher, Subcribers:
        self.task_api_pub = self.create_publisher(ApiRequest, "/task_api_requests", transient_qos)

        self.create_subscription(DeliveryRequest, "/delivery_requests",
                                 self.handle_request_cb,10)
        
        self.get_logger().info('Beginning workcells_manager node, shut down with CTRL-C')        


    def handle_request_cb(self, msg: DeliveryRequest):
        self.get_logger().info(f'Receive workcells_requests: delivery!')
        taskDict = {}
        # Set task request start time
        now = self.get_clock().now().to_msg()
        start_time = now.sec * 1000 + round(now.nanosec/10**6)
        if msg.mode.mode == DeliveryMode.MODE_PICKUP:
            taskDict = {
                "type": "dispatch_task_request",
                "request": {
                    "unix_millis_earliest_start_time": start_time,
                    "unix_millis_request_time": round(datetime.now().timestamp() * 1e3),
                    "priority": {
                        "type": "binary",
                        "value": 0
                    },
                    "category": "delivery",
                    "description": {
                        "pickup":{
                            "place": f"{msg.machine_name}--mcpickup",
                            "handler": f"{msg.machine_name}",
                            "payload": {"sku": "1", "quantity": 1}
                        },
                        "dropoff": {
                            "place": f"{msg.station_name}--dropoff",
                            "handler": f"{msg.station_name}",
                            "payload": {"sku": "1", "quantity": 1}
                        }
                    },
                    "requester": "admin"
                }
            }
        
        elif msg.mode.mode == DeliveryMode.MODE_DROPOFF:
            taskDict = {
                "type": "dispatch_task_request",
                "request": {
                    "unix_millis_earliest_start_time": start_time,
                    "unix_millis_request_time": round(datetime.now().timestamp() * 1e3),
                    "priority": {
                        "type": "binary",
                        "value": 0
                    },
                    "category": "delivery",
                    "description": {
                        "pickup":{
                            "place": f"{msg.station_name}--pickup",
                            "handler": f"{msg.station_name}",
                            "payload": {"sku": "1", "quantity": 1}
                        },
                        "dropoff": {
                            "place": f"{msg.machine_name}--mcdropoff",
                            "handler": f"{msg.machine_name}",
                            "payload": {"sku": "1", "quantity": 1}
                        }
                    },
                    "requester": "admin"
                }
            }
        else:
          return     

        taskJSON = json.dumps(taskDict)
        self.get_logger().info(f"JSON TASK: {taskJSON}")
        msgApi = ApiRequest()
        msgApi.request_id = str(uuid4())
        msgApi.json_msg = taskJSON
        self.task_api_pub.publish(msgApi)
        return

def main(args=None):
    # Init rclpy and adapter
    rclpy.init(args=args)

    workcells_handle = WorkcellHandle()
    rclpy.spin(workcells_handle)
    workcells_handle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()