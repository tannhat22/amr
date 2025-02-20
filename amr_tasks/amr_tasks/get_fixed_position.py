import ruamel.yaml
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
import rclpy.wait_for_message
from rmf_fleet_msgs.msg import FleetState, RobotState, Location


class FixedPosition(Node):

    def __init__(self, file_name, fleet_name, robot_name):

        super().__init__(f"get_fixed_position")

        # Params:
        self.declare_parameter("debug", True)

        self.debug = self.get_parameter("debug").value

        self.meters_on_pixels = {"L1": 0.03072}
        self.transform_levels = {
            "L1->L2": [0.99947, 1.16, 2.74, 0.00113],
            "L2->L1": [1.00053, -1.16, -2.74, -0.00113],
        }

        self.get_logger().info(f"meters_on_pixels: {self.meters_on_pixels}")
        self.get_logger().info(f"transform_levels: {self.transform_levels}")

        self.yaml = ruamel.yaml.YAML()
        self.yaml.default_flow_style = None

        self.file_name_ = file_name
        self.fleet_name = fleet_name
        self.robot_name = robot_name
        self.robot_location = Location()

        # Subscribers
        # self.create_subscription(FleetState, "fleet_states", self.fleet_states_cb, 10)

    def fleet_states_cb(self, msg: FleetState):
        if msg.name != self.fleet_name:
            return

        robot: RobotState
        for robot in msg.robots:
            if robot.name == self.robot_name:
                self.robot_location.x = robot.location.x
                self.robot_location.y = robot.location.y
                self.robot_location.yaw = robot.location.yaw
                self.robot_location.level_name = robot.location.level_name
                return

        self.get_logger().error(f"Not found robot [{self.robot_name}] in topic /fleet_states!")

    def convert_building_pixel_2_metter(
        self, from_point_x: float, from_point_y: float, level: str, y_flip: int
    ):
        if level == "L1":
            to_point_x = from_point_x
            to_point_y = from_point_y
        else:
            to_point_x = (
                self.transform_levels[f"{level}->L1"][0] * from_point_x
                + self.transform_levels[f"{level}->L1"][1]
            )
            to_point_y = (
                self.transform_levels[f"{level}->L1"][0] * from_point_y
                + self.transform_levels[f"{level}->L1"][2]
            )

        x_meter = to_point_x * self.meters_on_pixels[level]
        y_meter = to_point_y * self.meters_on_pixels[level] * y_flip

        return [x_meter, y_meter]

    def convert_building_metter_2_pixel(
        self, from_point_x: float, from_point_y: float, level: str, y_flip: int
    ):
        x_pixel = from_point_x / self.meters_on_pixels[level]
        y_pixel = from_point_y / (self.meters_on_pixels[level] * y_flip)

        if level == "L1":
            to_point_x_pixel = x_pixel
            to_point_y_pixel = y_pixel
        else:
            to_point_x_pixel = (
                x_pixel - self.transform_levels[f"{level}->L1"][1]
            ) / self.transform_levels[f"{level}->L1"][0]
            to_point_y_pixel = (
                y_pixel - self.transform_levels[f"{level}->L1"][2]
            ) / self.transform_levels[f"{level}->L1"][0]

        return [round(to_point_x_pixel, 1), round(to_point_y_pixel, 1)]

    def save_waypoints(self):
        isReceive, msg = rclpy.wait_for_message.wait_for_message(
            FleetState, self, "/fleet_states", time_to_wait=5
        )
        if not isReceive:
            self.get_logger().error("No msg fleet_state was publish!")
            return

        self.fleet_states_cb(msg=msg)

        position_name = input("Position name: ")

        position_pixels = self.convert_building_metter_2_pixel(
            self.robot_location.x, self.robot_location.y, self.robot_location.level_name, -1
        )

        with open(self.file_name_, "r") as file:
            data = self.yaml.load(file)

        if data is None:
            data = {}

        data[f"{position_name}"] = {}
        data[f"{position_name}"]["level"] = self.robot_location.level_name
        data[f"{position_name}"]["position(m)"] = [
            round(self.robot_location.x, 3),
            round(self.robot_location.y, 3),
            round(self.robot_location.yaw, 3),
        ]
        data[f"{position_name}"]["position(pixels)"] = position_pixels
        print(data[f"{position_name}"])

        with open(self.file_name_, "w") as f:
            self.yaml.dump(data, f)
            print("Written to file succesfully!")


def main(args=None):
    rclpy.init(args=args)

    file_name = "/home/tannhat/rmf_ws/src/amr/amr_tasks/fixed_position.yaml"
    fixed_position = FixedPosition(file_name, "amr_tp23", "amr001")
    fixed_position.save_waypoints()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fixed_position.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
