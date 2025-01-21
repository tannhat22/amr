# import ruamel.yaml
# import numpy as np
# import rclpy

# from rclpy.node import Node
# from rclpy.qos import qos_profile_system_default
# from rmf_fleet_msgs.msg import FleetState, RobotState, Location


# class FixedPosition(Node):

#     def __init__(self, file_name, fleet_name, robot_name):

#         super().__init__(f"get_fixed_position")

#         # Params:
#         self.declare_parameter("meters_on_pixels", [0.04108, 0.04109])
#         self.declare_parameter("debug", True)

#         meters_on_pixels = self.get_parameter("meters_on_pixels").value
#         self.debug = self.get_parameter("debug").value

#         self.scale = {}
#         for i in range(len(meters_on_pixels)):
#             level_name = f"L{i+1}"
#             self.scale.update({level_name: meters_on_pixels[i]})

#         self.get_logger().info(f"meters_on_pixels: {self.scale}")

#         self.yaml = ruamel.yaml.YAML()
#         self.yaml.default_flow_style = None

#         self.file_name_ = file_name
#         self.fleet_name = fleet_name
#         self.robot_name = robot_name
#         self.robot_location = Location()

#         # Subscribers
#         self.create_subscription(FleetState, "fleet_states", self.fleet_states_cb, 10)

#     def fleet_states_cb(self, msg: FleetState):
#         if msg.name != self.fleet_name:
#             return

#         robot: RobotState
#         for robot in msg.robots:
#             if robot.name == self.robot_name:
#                 self.robot_location.x = robot.location.x
#                 self.robot_location.y = robot.location.y
#                 self.robot_location.yaw = robot.location.yaw
#                 self.robot_location.level_name = robot.location.level_name

#     def transform_position(self, x, y, scale, translation, rotation):
#         # used 4 measurements to estimate meters/pixel as 0.03292
#         # used 4 measurements to estimate meters/pixel as 0.04996
#         # transform 0->1: scale = 0.66000 translation = (-852.34, -274.79) rotation = -0.00038
#         # transform 1->0: scale = 1.51520 translation = (1291.53, 415.82) rotation = 0.00038
#         # to_point_x = scale * x + translation[0]
#         # to_point_y = scale * y + translation[1]
#         # x_m = to_point_x * scale,
#         # y_m = to_point_y * scale * -1
#         # x = (pixel_x * 1.51520 + 1291.53)*  0.04996

#         """
#         Chuyển đổi vị trí dựa trên scale, translation, và rotation.

#         Args:
#             x (float): Tọa độ x ban đầu.
#             y (float): Tọa độ y ban đầu.
#             scale (float): Hệ số co dãn.
#             translation (tuple): Vector tịnh tiến (tx, ty).
#             rotation (float): Góc quay (radian).

#         Returns:
#             tuple: Tọa độ (x', y') sau khi chuyển đổi.
#         """
#         # Tạo ma trận scale
#         scale_matrix = np.array([[scale, 0], [0, scale]])

#         # Tạo ma trận rotation
#         rotation_matrix = np.array(
#             [[np.cos(rotation), -np.sin(rotation)], [np.sin(rotation), np.cos(rotation)]]
#         )

#         # Tích hợp scale và rotation
#         transform_matrix = np.dot(rotation_matrix, scale_matrix)

#         # Áp dụng phép biến đổi
#         original_position = np.array([x, y])
#         transformed_position = np.dot(transform_matrix, original_position) + np.array(translation)

#         return transformed_position[0], transformed_position[1]

#     def save_waypoints(self):

#         position_name = input("Position name: ")

#         position_pixels = []
#         position_pixels[0] = self.robot_location.x * self.scale

#         with open(self.file_name_, "r") as file:
#             data = self.yaml.load(file)

#         data[f"{position_name}"] = {}
#         data[f"{position_name}"]["level"] = self.robot_location.level_name
#         data[f"{position_name}"]["position(m)"] = [
#             round(self.robot_location.x, 3),
#             round(self.robot_location.y, 3),
#             round(self.robot_location.yaw, 3),
#         ]
#         data[f"{position_name}"]["position(pixels)"] = []
#         print(data[f"{position_name}"])

#         with open(self.file_name_, "w") as f:
#             self.yaml.dump(data, f)
#             print("Written to file succesfully!")


# if __name__ == "__main__":

#     file_name = (
#         "/home/amr/catkin_ws/src/amr_v3/amr_v3_waypoint_generator/config/fixed_position.yaml"
#     )

#     try:
#         fixed_position = FixedPosition(file_name)
#         fixed_position.save_waypoints()

#     except rospy.ROSInterruptException:
#         pass
