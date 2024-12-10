import sys
import argparse
import yaml
import math

import rclpy
from enum import IntEnum
from rclpy.node import Node

from rclpy.qos import qos_profile_system_default
from rmf_fleet_msgs.msg import FleetState, RobotMode, RobotState, ModeRequest, Location


class State:
    def __init__(self, last_mode_request: RobotMode = None):
        self.last_mode_request = last_mode_request
        self.wait_HID = None
        self.wait_LID = []


class PriotityCode(IntEnum):
    DEST_TOO_NEAR_POSSITION = 1
    BATTERY_PRIORITY = 2
    BATTERY_PRIORITY_THRESHOLD = 3
    BATTERY_PRIORITY_THRESHOLD_AND_PATH_EQUAL = 4
    BATTERY_PRIORITY_THRESHOLD_AND_PATH_SHORTER = 5
    PATH_DISTANCE_PRIORITY = 6


class FleetConflictsHandle(Node):
    robots: dict[str, State]

    def __init__(self, config):
        self.config = config

        super().__init__(f"fleet_conflicts_handle")

        # Params:
        self.declare_parameter("width_conflict", 1.0)
        self.declare_parameter("height_conflict", 2.0)
        self.declare_parameter("front_extension", 1.0)
        self.declare_parameter("debug", True)

        self.width_conflict = self.get_parameter("width_conflict").value
        self.height_conflict = self.get_parameter("height_conflict").value
        self.front_extension = self.get_parameter("front_extension").value
        self.debug = self.get_parameter("debug").value

        if self.debug:
            self.get_logger().info(f"width_conflict: {self.width_conflict}")
            self.get_logger().info(f"height_conflict: {self.height_conflict}")
            self.get_logger().info(f"front_extension: {self.front_extension}")

        self.robots = {}
        for robot_name, robot_config in self.config["rmf_fleet"]["robots"].items():
            self.robots[robot_name] = State()
        assert len(self.robots) > 0

        self.recharge_threshold = self.config["rmf_fleet"]["recharge_threshold"]
        self.current_cmd_id = ""

        self.mode_request_pub = self.create_publisher(
            ModeRequest,
            "action_execution_notice",
            qos_profile=qos_profile_system_default,
        )

        self.create_subscription(
            FleetState,
            "fleet_states",
            self.fleet_states_cb,
            100,
        )

    def dist(self, A: Location, B: Location):
        """Euclidian distance between A(x,y) and B(x,y)"""
        return math.sqrt((A.x - B.x) ** 2 + (A.y - B.y) ** 2)

    def mode_request(self, fleet_name: str, robot_name: str, mode: int):
        msg = ModeRequest()
        msg.fleet_name = fleet_name
        msg.robot_name = robot_name
        msg.mode.mode = mode
        self.mode_request_pub.publish(msg)

    # Check priority between robot A and B with state of two robots
    # return [highPriority, lowPriority]
    def check_priority(self, A: int, B: int, robot_A: RobotState, robot_B: RobotState):
        destA = None
        destB = None
        result = [A, B]
        code = None
        posCurrent_A = robot_A.location
        posCurrent_B = robot_B.location

        if len(robot_A.path) > 0:
            destA = robot_A.path[-1]
        if len(robot_B.path) > 0:
            destB = robot_B.path[-1]

        # Kiểm tra đích đến của robot có nằm quá gần vị trí của robot còn lại không:
        if destA is not None and self.dist(destA, posCurrent_B) < 1.0:
            code = PriotityCode.DEST_TOO_NEAR_POSSITION
            result = [B, A]
        elif destB is not None and self.dist(destB, posCurrent_A) < 1.0:
            code = PriotityCode.DEST_TOO_NEAR_POSSITION
            result = [A, B]
        # Kiểm tra cả hai robot đều đã dưới mức ngưỡng sạc:
        elif (
            robot_A.battery_percent <= self.recharge_threshold
            and robot_B.battery_percent <= self.recharge_threshold
        ):
            robotA_path_dis = self.calc_path_distance(robot_A.location, robot_A.path)
            robotB_path_dis = self.calc_path_distance(robot_B.location, robot_B.path)
            if robotA_path_dis == robotB_path_dis:
                code = PriotityCode.BATTERY_PRIORITY_THRESHOLD_AND_PATH_EQUAL
                if robot_A.battery_percent <= robot_B.battery_percent:
                    result = [A, B]
                else:
                    result = [B, A]
            elif robotA_path_dis > robotB_path_dis:
                code = PriotityCode.BATTERY_PRIORITY_THRESHOLD_AND_PATH_SHORTER
                result = [A, B]
            else:
                code = PriotityCode.BATTERY_PRIORITY_THRESHOLD_AND_PATH_SHORTER
                result = [B, A]
        elif robot_A.battery_percent <= self.recharge_threshold:
            code = PriotityCode.BATTERY_PRIORITY_THRESHOLD
            result = [A, B]
        elif robot_B.battery_percent <= self.recharge_threshold:
            code = PriotityCode.BATTERY_PRIORITY_THRESHOLD
            result = [B, A]
        elif robot_A.battery_percent / robot_B.battery_percent > 1.1:
            code = PriotityCode.BATTERY_PRIORITY
            result = [B, A]
        elif robot_B.battery_percent / robot_A.battery_percent > 1.1:
            code = PriotityCode.BATTERY_PRIORITY
            result = [A, B]
        else:
            robotA_path_dis = self.calc_path_distance(robot_A.location, robot_A.path)
            robotB_path_dis = self.calc_path_distance(robot_B.location, robot_B.path)

            code = PriotityCode.PATH_DISTANCE_PRIORITY
            if robotA_path_dis is None:
                self.get_logger().warn(f"[{robot_A.name}] don't have path, please check!")
                return [B, A]
            elif robotB_path_dis is None:
                self.get_logger().warn(f"[{robot_B.name}] don't have path, please check!")
                return [A, B]

            if robotA_path_dis >= robotB_path_dis:
                result = [A, B]
            else:
                result = [B, A]

        if result[0] == A:
            self.get_logger().warn(
                f"[{robot_B.name}] will pause for conflicts handle (code: {code})!"
            )
        else:
            self.get_logger().warn(
                f"[{robot_A.name}] will pause for conflicts handle (code: {code})!"
            )
        return result

    # Hàm tính vector hướng từ tọa độ hiện tại đến điểm đến
    def vector_direction(self, start: Location, end: Location):
        return (end.x - start.x, end.y - start.y)

    # Hàm tính độ lớn của vector
    def vector_magnitude(self, vector):
        return math.sqrt(vector[0] ** 2 + vector[1] ** 2)

    # Hàm tính góc giữa hai vector (trả về giá trị bằng độ)
    def angle_between_vectors(self, v1, v2):
        dot_product = v1[0] * v2[0] + v1[1] * v2[1]  # Tích vô hướng của hai vector
        magnitude_v1 = self.vector_magnitude(v1)
        magnitude_v2 = self.vector_magnitude(v2)

        if magnitude_v1 == 0 or magnitude_v2 == 0:
            raise ValueError("Độ lớn của vector không được bằng 0.")

        # cos(theta) = (v1 . v2) / (|v1| * |v2|)
        cos_theta = dot_product / (magnitude_v1 * magnitude_v2)

        # Tránh sai số số học
        cos_theta = min(1, max(cos_theta, -1))

        # Tính góc theta bằng radian và chuyển sang độ
        theta_radians = math.acos(cos_theta)
        # theta_degrees = math.degrees(theta_radians)

        return theta_radians

    def calculate_rectangle_corners(self, position: Location, width, height, front_extension):
        # Tính cosine và sine của yaw
        cos_yaw = math.cos(position.yaw)
        sin_yaw = math.sin(position.yaw)

        # Tính tọa độ 4 điểm với chiều dài mở rộng phía trước
        corners = {
            "front_left": (
                position.x - (width / 2) * cos_yaw - (height / 2) * sin_yaw,
                position.y - (width / 2) * sin_yaw + (height / 2) * cos_yaw,
            ),
            "front_right": (
                position.x + (width / 2) * cos_yaw - (height / 2) * sin_yaw,
                position.y + (width / 2) * sin_yaw + (height / 2) * cos_yaw,
            ),
            "back_left": (
                position.x - (width / 2) * cos_yaw + (height / 2) * sin_yaw,
                position.y - (width / 2) * sin_yaw - (height / 2) * cos_yaw,
            ),
            "back_right": (
                position.x + (width / 2) * cos_yaw + (height / 2) * sin_yaw,
                position.y + (width / 2) * sin_yaw - (height / 2) * cos_yaw,
            ),
        }

        # Tính toán tọa độ cho các điểm phía trước với chiều dài mở rộng
        front_extension_corners = {
            "front_left": (
                corners["front_left"][0] + front_extension * cos_yaw,
                corners["front_left"][1] + front_extension * sin_yaw,
            ),
            "front_right": (
                corners["front_right"][0] + front_extension * cos_yaw,
                corners["front_right"][1] + front_extension * sin_yaw,
            ),
        }

        return {
            "front_left": front_extension_corners["front_left"],
            "front_right": front_extension_corners["front_right"],
            "back_left": corners["back_left"],
            "back_right": corners["back_right"],
        }

    def is_rectangles_overlap(self, rect1: dict, rect2: dict):
        # Lấy tọa độ của các điểm
        rect1_points = [
            rect1["front_left"],
            rect1["front_right"],
            rect1["back_left"],
            rect1["back_right"],
        ]

        rect2_points = [
            rect2["front_left"],
            rect2["front_right"],
            rect2["back_left"],
            rect2["back_right"],
        ]

        # Tìm min và max cho hình chữ nhật 1
        x1_min = min(point[0] for point in rect1_points)
        x1_max = max(point[0] for point in rect1_points)
        y1_min = min(point[1] for point in rect1_points)
        y1_max = max(point[1] for point in rect1_points)

        # Tìm min và max cho hình chữ nhật 2
        x2_min = min(point[0] for point in rect2_points)
        x2_max = max(point[0] for point in rect2_points)
        y2_min = min(point[1] for point in rect2_points)
        y2_max = max(point[1] for point in rect2_points)

        # Kiểm tra điều kiện không chồng lấn
        if x1_max < x2_min or x2_max < x1_min or y1_max < y2_min or y2_max < y1_min:
            return False  # Không chồng lấn

        return True  # Có chồng lấn

    def calc_path_distance(self, position: Location, path: list[Location]):
        a = len(path)
        if a == 0:
            return None
        else:
            pointDest = path[0]
            dist = self.dist(position, pointDest)
            for i in range(a - 1):
                pointA = path[i]
                pointB = path[i + 1]
                dist += self.dist(pointA, pointB)
            return dist

    def fleet_states_cb(self, msg: FleetState):
        dataRobot: list[RobotState]

        fleetName = msg.name
        dataRobot = msg.robots
        dataLen = len(dataRobot)
        for i in range(dataLen):
            robot1Name = dataRobot[i].name
            if robot1Name not in self.robots:
                continue

            robot1Mode = dataRobot[i].mode.mode
            if (
                robot1Mode == RobotMode.MODE_IDLE
                or robot1Mode == RobotMode.MODE_CHARGING
                or robot1Mode == RobotMode.MODE_EMERGENCY
                or robot1Mode == RobotMode.MODE_REQUEST_ERROR
            ):
                if len(dataRobot[i].path) == 0:
                    if self.robots[robot1Name].wait_HID is not None:
                        self.robots[self.robots[robot1Name].wait_HID].wait_LID.remove(robot1Name)
                        self.robots[robot1Name].wait_HID = None
                        self.robots[robot1Name].last_mode_request = None
                        self.get_logger().info(
                            f"Robot [{robot1Name}] is not moving or pause will reset state!"
                        )

                    if len(self.robots[robot1Name].wait_LID) != 0:
                        for robot in self.robots[robot1Name].wait_LID:
                            self.mode_request(
                                fleet_name=fleetName,
                                robot_name=robot,
                                mode=RobotMode.MODE_MOVING,
                            )
                            self.robots[robot1Name].wait_LID.remove(robot)
                            self.robots[robot].wait_HID = None
                            self.robots[robot].last_mode_request = None
                            self.get_logger().info(
                                f"Publish RESUME_ACTION for [{robot}] from conflicts handle!"
                            )
                continue

            for j in range(dataLen):
                if i == j:
                    continue

                robot2Name = dataRobot[j].name
                if robot2Name not in self.robots:
                    continue

                robot2Mode = dataRobot[j].mode.mode
                # Kiểm tra xem 2 robot này có cùng tầng không:
                if dataRobot[i].location.level_name == dataRobot[j].location.level_name:
                    posA = dataRobot[i].location
                    posB = dataRobot[j].location
                    if len(dataRobot[i].path) > 0 and len(dataRobot[j].path) > 0:
                        rectA = self.calculate_rectangle_corners(
                            posA,
                            self.width_conflict,
                            self.height_conflict,
                            self.front_extension,
                        )
                        rectB = self.calculate_rectangle_corners(
                            posB,
                            self.width_conflict,
                            self.height_conflict,
                            self.front_extension,
                        )
                        # Kiểm tra có xự xâm lấn vùng conflict theo hướng robot hiện tại hay không
                        if self.is_rectangles_overlap(rectA, rectB):
                            if self.debug:
                                self.get_logger().info(
                                    f"Detect overlap of two robot: [{robot1Name}] and [{robot2Name}]"
                                )
                            # Kiểm tra xem robot1 có dang di chuyển hay không:
                            if robot1Mode == RobotMode.MODE_MOVING:
                                # Nếu robot2 cũng đang di chuyển thì phải kiểm tra mức độ ưu tiên,
                                # robot nào có độ ưu tiên thấp hơn sẽ phải chuyển sang chế độ tạm dừng
                                if robot2Mode == RobotMode.MODE_MOVING:
                                    destA = dataRobot[i].path[-1]
                                    destB = dataRobot[j].path[-1]
                                    vecA = self.vector_direction(posA, destA)
                                    vecB = self.vector_direction(posB, destB)

                                    theta = self.angle_between_vectors(vecA, vecB)
                                    if self.debug:
                                        self.get_logger().info(
                                            f"Moving_Angle between [{robot1Name}] and [{robot2Name}]: {theta}"
                                        )
                                    if theta > 3.0:
                                        if (
                                            self.robots[dataRobot[i].name].last_mode_request is None
                                            and self.robots[dataRobot[j].name].last_mode_request
                                            is None
                                        ):
                                            # Lựa chọn robot nào sẽ có độ ưu tiên cao hơn
                                            prioHID, prioLID = self.check_priority(
                                                i, j, dataRobot[i], dataRobot[j]
                                            )
                                            if (
                                                self.robots[
                                                    dataRobot[prioLID].name
                                                ].last_mode_request
                                                != RobotMode.MODE_PAUSED
                                            ):
                                                # Yêu cầu robot không được ưu tiên sẽ chuyển sang MODE_PAUSED
                                                self.mode_request(
                                                    fleet_name=fleetName,
                                                    robot_name=dataRobot[prioLID].name,
                                                    mode=RobotMode.MODE_PAUSED,
                                                )
                                                self.robots[
                                                    dataRobot[prioLID].name
                                                ].last_mode_request = RobotMode.MODE_PAUSED
                                                self.robots[dataRobot[prioLID].name].wait_HID = (
                                                    dataRobot[prioHID].name
                                                )
                                                self.robots[
                                                    dataRobot[prioHID].name
                                                ].wait_LID.append(dataRobot[prioLID].name)
                                                self.get_logger().warn(
                                                    f"Publish PAUSED_ACTION for [{dataRobot[prioLID].name}] (waiting [{dataRobot[prioHID].name}])!"
                                                )

                                # Nếu robot2 dang ở chế độ tạm dừng bởi wait_HID khác thì robot1 cũng
                                # sẽ chuyển sang chế độ tạm dừng để tránh xung đột với wait_HID của robot2
                                elif (
                                    robot2Mode == RobotMode.MODE_PAUSED
                                    and self.robots[robot2Name].wait_HID is not None
                                    and self.robots[robot2Name].wait_HID != robot1Name
                                    and self.robots[robot1Name].last_mode_request
                                    != RobotMode.MODE_PAUSED
                                ):
                                    self.mode_request(
                                        fleet_name=fleetName,
                                        robot_name=robot1Name,
                                        mode=RobotMode.MODE_PAUSED,
                                    )
                                    self.robots[robot1Name].last_mode_request = (
                                        RobotMode.MODE_PAUSED
                                    )
                                    self.robots[robot1Name].wait_HID = robot2Name
                                    self.robots[robot2Name].wait_LID.append(robot1Name)
                                    self.get_logger().warn(
                                        f"Publish PAUSED_ACTION for [{robot1Name}] (waiting [{robot2Name}])!"
                                    )
                        else:
                            # Không có sự xâm lấn, nếu robot1 đang chờ robot2 hãy giải phóng robot1
                            if (
                                robot1Mode == RobotMode.MODE_PAUSED
                                and self.robots[robot1Name].wait_HID == robot2Name
                                and self.robots[robot1Name].last_mode_request is not None
                            ):
                                self.mode_request(
                                    fleet_name=fleetName,
                                    robot_name=robot1Name,
                                    mode=RobotMode.MODE_MOVING,
                                )
                                self.robots[robot2Name].wait_LID.remove(robot1Name)
                                self.robots[robot1Name].last_mode_request = None
                                self.robots[robot1Name].wait_HID = None
                                self.get_logger().info(
                                    f"Publish RESUME_ACTION for [{robot1Name}] from conflicts handle!"
                                )

                # Nếu robot khác tầng với nhau, hãy kiểm tra nếu robot1
                # đang tạm dừng để chờ robot2 hãy giải phóng robot1
                elif (
                    robot1Mode == RobotMode.MODE_PAUSED
                    and self.robots[robot1Name].wait_HID == robot2Name
                    and self.robots[robot1Name].last_mode_request is not None
                ):
                    self.mode_request(
                        fleet_name=fleetName,
                        robot_name=robot1Name,
                        mode=RobotMode.MODE_MOVING,
                    )
                    self.robots[robot2Name].wait_LID.remove(robot1Name)
                    self.robots[robot1Name].last_mode_request = None
                    self.robots[robot1Name].wait_HID = None
                    self.get_logger().info(
                        f"Publish RESUME_ACTION for [{robot1Name}] from conflicts handle!"
                    )


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def main(argv=sys.argv):
    # Init rclpy
    rclpy.init(args=argv)
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="fleet_conflicts_handle",
        description="Configure and spin up the fleet conflict handle",
    )
    parser.add_argument(
        "-c",
        "--config_file",
        type=str,
        required=True,
        help="Path to the config.yaml file",
    )
    args = parser.parse_args(args_without_ros[1:])
    print(f"Starting fleet conflicts handle...")

    with open(args.config_file, "r") as f:
        config = yaml.safe_load(f)

    fleet_conflicts_handle = FleetConflictsHandle(config)
    rclpy.spin(fleet_conflicts_handle)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fleet_conflicts_handle.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
