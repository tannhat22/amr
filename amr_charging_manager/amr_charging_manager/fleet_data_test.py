import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
)
from std_msgs.msg import String
import json
import time
import random


class FleetStatePublisher(Node):
    def __init__(self):
        super().__init__("fleet_state_publisher")

        transient_qos = QoSProfile(
            # history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.publisher = self.create_publisher(
            String, "/fleet_state_update", qos_profile=transient_qos
        )
        self.update_interval = 20.0  # giây, cấu hình khoảng thời gian random pin
        self.timer = self.create_timer(1.0, self.timer_callback)  # timer chạy 1s/lần

        self.last_update_time = 0.0
        self.battery_values = {}  # lưu pin hiện tại của từng robot

        # Khởi tạo pin mặc định cho robot
        self.init_battery_values()

    def init_battery_values(self):
        robots = [
            ("amr_tp2", "amr004"),
            ("amr_tp3", "amr001"),
            ("amr_tp3", "amr002"),
            ("amr_tp3", "amr003"),
            ("amr_tp3", "amr005"),
        ]
        for fleet, robot in robots:
            self.battery_values[(fleet, robot)] = random.uniform(0.05, 1.0)

    def random_battery(self, fleet, robot):
        # Cập nhật pin ngẫu nhiên cho robot
        new_battery = random.uniform(0.05, 1.0)
        self.battery_values[(fleet, robot)] = round(new_battery, 2)
        return self.battery_values[(fleet, robot)]

    def timer_callback(self):
        now = time.time()
        now_ms = int(now * 1000)

        # Cập nhật pin theo khoảng thời gian cấu hình
        if now - self.last_update_time >= self.update_interval:
            self.get_logger().info("Updating battery values...")
            # Cập nhật lại pin cho tất cả robot
            for key in self.battery_values.keys():
                self.random_battery(*key)
            self.last_update_time = now
            print("Updated battery values: ")
            for key, value in self.battery_values.items():
                self.get_logger().info(f"Robot [{key[1]}] battery: {value}")
            print("//////////////////////////////////")

        fleet_state_tp2 = {
            "name": "amr_tp2",
            "robots": {
                "amr004": {
                    "name": "amr004",
                    "status": "idle",
                    "task_id": "",
                    "unix_millis_time": now_ms,
                    "location": {"x": 1.0, "y": 2.0, "yaw": 0.0, "map": "L1"},
                    "battery": self.battery_values[("amr_tp2", "amr004")],
                    "mutex_groups": {"locked": [], "requesting": []},
                }
            },
        }

        fleet_state_tp3 = {
            "name": "amr_tp3",
            "robots": {
                "amr001": {
                    "name": "amr001",
                    "status": "idle",
                    "task_id": "",
                    "unix_millis_time": now_ms,
                    "location": {
                        "x": 107.0,
                        "y": -46.0,
                        "yaw": 0.0,
                        "map": "L2",
                    },
                    "battery": self.battery_values[("amr_tp3", "amr001")],
                    "mutex_groups": {"locked": [], "requesting": []},
                },
                "amr002": {
                    "name": "amr002",
                    "status": "idle",
                    "task_id": "",
                    "unix_millis_time": now_ms,
                    "location": {
                        "x": 100.0,
                        "y": -47.0,
                        "yaw": 0.0,
                        "map": "L2",
                    },
                    "battery": self.battery_values[("amr_tp3", "amr002")],
                    "mutex_groups": {"locked": [], "requesting": []},
                },
                "amr003": {
                    "name": "amr003",
                    "status": "idle",
                    "task_id": "",
                    "unix_millis_time": now_ms,
                    "location": {"x": 1.0, "y": 2.0, "yaw": 0.0, "map": "L2"},
                    "battery": self.battery_values[("amr_tp3", "amr003")],
                    "mutex_groups": {"locked": [], "requesting": []},
                },
                "amr005": {
                    "name": "amr005",
                    "status": "idle",
                    "task_id": "",
                    "unix_millis_time": now_ms,
                    "location": {"x": 1.0, "y": 2.0, "yaw": 0.0, "map": "L2"},
                    "battery": self.battery_values[("amr_tp3", "amr005")],
                    "mutex_groups": {"locked": [], "requesting": []},
                },
            },
        }

        for fleet_state in [fleet_state_tp2, fleet_state_tp3]:
            msg = String()
            msg.data = json.dumps({"data": fleet_state})
            self.publisher.publish(msg)
            time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)
    node = FleetStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
