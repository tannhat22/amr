#!/usr/bin/env python3

from uuid import uuid4
import json
import rclpy
from rclpy.node import Node

from rmf_task_msgs.msg import ApiRequest, ApiResponse
from machine_fleet_msgs.msg import DeliveryMode, DeliveryRequest

class WorkcellHandle(Node):

    def __init__(self):
        super().__init__("workcells_manager")

        # Publisher, Subcribers:
        self.task_api_pub = self.create_publisher(ApiRequest, "/task_api_requests", 10)

        self.create_subscription(DeliveryRequest, "/delivery_requests",
                                 self.handle_request_cb,10)
                
        # Variables:

        self.get_logger().info('Beginning workcells_manager node, shut down with CTRL-C')        

    def handle_request_cb(self, msg: DeliveryRequest):
        self.get_logger().info(f'Receive workcells_requests: delivery!')
        taskDict = {}
        if msg.mode.mode == DeliveryMode.MODE_PICKUP:
            taskDict= {
                "type": "dispatch_task_request",
                "request": {
                    "category": "delivery",
                    "description": {
                    "pickup": {
                        "place": f"{msg.machine_name}--mcpickup",
                        "handler": f"{msg.machine_name}",
                        "payload": [
                        {"sku": "cart",
                        "quantity": 1}
                        ]
                    },
                    "dropoff": {
                        "place": f"{msg.station_name}--dropoff",
                        "handler": f"{msg.station_name}",
                        "payload": [
                        {"sku": "cart",
                        "quantity": 1}
                        ]
                    }
                    }
                }
            }
        
        elif msg.mode.mode == DeliveryMode.MODE_DROPOFF:
            taskDict= {
                "type": "dispatch_task_request",
                "request": {
                    "category": "delivery",
                    "description": {
                    "pickup": {
                        "place": f"{msg.station_name}--pickup",
                        "handler": f"{msg.station_name}",
                        "payload": [
                        {"sku": "cart",
                        "quantity": 1}
                        ]
                    },
                    "dropoff": {
                        "place": f"{msg.machine_name}--mcdropoff",
                        "handler": f"{msg.machine_name}",
                        "payload": [
                        {"sku": "cart",
                        "quantity": 1}
                        ]
                    }
                    }
                }
            }
        else:
          return     

        taskJSON = json.dumps(taskDict)
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
