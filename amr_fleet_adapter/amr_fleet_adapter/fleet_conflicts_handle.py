import sys
import argparse
import yaml
import math

# import threading
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node

# from rclpy.executors import MultiThreadedExecutor
# from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from enum import IntEnum
from rclpy.qos import qos_profile_system_default
from rmf_fleet_msgs.msg import FleetState, RobotMode, RobotState, ModeRequest, Location


class State:
    def __init__(
        self,
        fleet_name: str = "",
        state: RobotState = None,
        last_mode_request: RobotMode = None,
    ):
        self.fleet_name = fleet_name
        self.state = state
        self.last_mode_request = last_mode_request
        self.wait_HID = None
        self.wait_LID = []


class Rectangle:
    def __init__(self, front_left, front_right, back_left, back_right):
        self.front_left = front_left
        self.front_right = front_right
        self.back_left = back_left
        self.back_right = back_right
        self.corners = [front_left, front_right, back_right, back_left]


class PriotityCode(IntEnum):
    DEST_TOO_NEAR_POSSITION = 1
    BATTERY_PRIORITY = 2
    BATTERY_PRIORITY_THRESHOLD = 3
    BATTERY_PRIORITY_THRESHOLD_AND_PATH_EQUAL = 4
    BATTERY_PRIORITY_THRESHOLD_AND_PATH_SHORTER = 5
    PATH_DISTANCE_PRIORITY = 6


class FleetConflictsHandle(Node):
    robots: dict[str, State]

    def __init__(self, configs):
        super().__init__(f"fleet_conflicts_handle")

        # Callback groups:
        # robot_cb_group = ReentrantCallbackGroup()

        # Params:
        self.declare_parameter("update_frequency", 5.0)
        self.declare_parameter("width_conflict", 1.0)
        self.declare_parameter("height_conflict", 2.0)
        self.declare_parameter("front_extension", 1.0)
        self.declare_parameter("debug", True)

        self.update_frequency = self.get_parameter("update_frequency").value
        self.width_conflict = self.get_parameter("width_conflict").value
        self.height_conflict = self.get_parameter("height_conflict").value
        self.front_extension = self.get_parameter("front_extension").value
        self.debug = self.get_parameter("debug").value

        if self.debug:
            self.get_logger().info(f"update_frequency: {self.update_frequency}")
            self.get_logger().info(f"width_conflict: {self.width_conflict}")
            self.get_logger().info(f"height_conflict: {self.height_conflict}")
            self.get_logger().info(f"front_extension: {self.front_extension}")

            # Chart
            self.fig, self.axs = plt.subplots(
                2, 1, figsize=(8, 6)
            )  # Tạo một figure và axes cho biểu đồ
            self.levels_ax = {"L1": self.axs[0], "L2": self.axs[1]}
            plt.ion()  # Bật chế độ interactive để vẽ động (tự động cập nhật)
            plt.show()

        self.detect_robot = False
        self.robots = {}
        for config in configs:
            if config == "":
                continue

            for robot_name, robot_config in config["rmf_fleet"]["robots"].items():
                self.robots[robot_name] = State(fleet_name=config["rmf_fleet"]["name"])
            self.robots_length = len(self.robots)
        assert self.robots_length > 0

        self.recharge_threshold = config["rmf_fleet"]["recharge_threshold"]
        update_period = 1.0 / self.update_frequency

        # Threading variables
        # self._lock = threading.Lock()

        self.mode_request_pub = self.create_publisher(
            ModeRequest,
            "action_execution_notice",
            qos_profile=qos_profile_system_default,
        )

        self.create_subscription(FleetState, "fleet_states", self.fleet_states_cb, 100)

        self.create_timer(update_period, self._conflict_handle_cb)

    def dist(self, A: Location, B: Location):
        """Euclidian distance between A(x,y) and B(x,y)"""
        return math.sqrt((A.x - B.x) ** 2 + (A.y - B.y) ** 2)

    def plot_rectangle(self, ax, corners, label):
        x_vals = [corner[0] for corner in corners]
        y_vals = [corner[1] for corner in corners]
        x_vals.append(x_vals[0])
        y_vals.append(y_vals[0])
        ax.plot(x_vals, y_vals)

    def plot_yaw_vector(self, ax, position: Location, length=1):
        end_x = position.x + length * math.cos(position.yaw)
        end_y = position.y + length * math.sin(position.yaw)
        ax.arrow(
            position.x,
            position.y,
            end_x - position.x,
            end_y - position.y,
            head_width=0.5,
            head_length=0.7,
            fc="red",
            ec="red",
            label="Yaw Direction",
        )

    def update_plot(self, robots_on_level: dict[str, State]):
        # Tạo hoặc vẽ lại biểu đồ cho vị trí các robot
        # Lặp qua các robot và vẽ biểu đồ cho chúng
        for level, robots in robots_on_level.items():
            ax = self.levels_ax[level]
            ax.clear()
            ax.set_title(f"LEVELS: [{level}]")
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            # ax.set_xlim(-200, 200)
            # ax.set_ylim(-200, 200)
            ax.set_aspect("equal", adjustable="box")
            ax.grid(True)

            for robot in robots:
                if robot.state is not None:
                    x = robot.state.location.x
                    y = robot.state.location.y
                    name = robot.state.name

                    # Vẽ điểm của robot
                    ax.scatter(x, y, label=name)

                    # Vẽ hình chữ nhật (hoặc mô hình di chuyển của robot)
                    rect = self.calculate_rectangle(
                        robot.state.location,
                        self.width_conflict,
                        self.height_conflict,
                        self.front_extension,
                    )
                    self.plot_rectangle(ax, rect.corners, name)
                    self.plot_yaw_vector(ax, robot.state.location, 1)

            ax.legend()

        self.fig.canvas.draw()  # Cập nhật biểu đồ
        self.fig.canvas.flush_events()  # Đảm bảo sự kiện được thực thi

    def mode_request(self, fleet_name: str, robot_name: str, mode: int):
        msg = ModeRequest()
        msg.fleet_name = fleet_name
        msg.robot_name = robot_name
        msg.mode.mode = mode
        self.mode_request_pub.publish(msg)

    # Check priority between robot A and B with state of two robots
    # return highPriority
    def check_priority(self, robot_A: State, robot_B: State):
        destA = None
        destB = None
        result = robot_A.state.name
        code = None
        posCurrent_A = robot_A.state.location
        posCurrent_B = robot_B.state.location

        if len(robot_A.state.path) > 0:
            destA = robot_A.state.path[-1]
        if len(robot_B.state.path) > 0:
            destB = robot_B.state.path[-1]

        # Kiểm tra đích đến của robot có nằm quá gần vị trí của robot còn lại không:
        if destA is not None and self.dist(destA, posCurrent_B) < 1.0:
            code = PriotityCode.DEST_TOO_NEAR_POSSITION
            result = robot_B.state.name
        elif destB is not None and self.dist(destB, posCurrent_A) < 1.0:
            code = PriotityCode.DEST_TOO_NEAR_POSSITION
            result = robot_A.state.name
        # Kiểm tra cả hai robot đều đã dưới mức ngưỡng sạc:
        elif (
            robot_A.state.battery_percent <= self.recharge_threshold
            and robot_B.state.battery_percent <= self.recharge_threshold
        ):
            robotA_path_dis = self.calc_path_distance(robot_A.state.location, robot_A.state.path)
            robotB_path_dis = self.calc_path_distance(robot_B.state.location, robot_B.state.path)
            if robotA_path_dis == robotB_path_dis:
                code = PriotityCode.BATTERY_PRIORITY_THRESHOLD_AND_PATH_EQUAL
                if robot_A.state.battery_percent <= robot_B.state.battery_percent:
                    result = robot_A.state.name
                else:
                    result = robot_B.state.name
            elif robotA_path_dis > robotB_path_dis:
                code = PriotityCode.BATTERY_PRIORITY_THRESHOLD_AND_PATH_SHORTER
                result = robot_A.state.name
            else:
                code = PriotityCode.BATTERY_PRIORITY_THRESHOLD_AND_PATH_SHORTER
                result = robot_B.state.name
        elif robot_A.state.battery_percent <= self.recharge_threshold:
            code = PriotityCode.BATTERY_PRIORITY_THRESHOLD
            result = robot_A.state.name
        elif robot_B.state.battery_percent <= self.recharge_threshold:
            code = PriotityCode.BATTERY_PRIORITY_THRESHOLD
            result = robot_B.state.name
        elif robot_A.state.battery_percent / robot_B.state.battery_percent > 1.1:
            code = PriotityCode.BATTERY_PRIORITY
            result = robot_B.state.name
        elif robot_B.state.battery_percent / robot_A.state.battery_percent > 1.1:
            code = PriotityCode.BATTERY_PRIORITY
            result = robot_A.state.name
        else:
            robotA_path_dis = self.calc_path_distance(robot_A.state.location, robot_A.state.path)
            robotB_path_dis = self.calc_path_distance(robot_B.state.location, robot_B.state.path)

            code = PriotityCode.PATH_DISTANCE_PRIORITY
            if robotA_path_dis is None:
                self.get_logger().warn(f"[{robot_A.state.name}] don't have path, please check!")
                return robot_B.state.name
            elif robotB_path_dis is None:
                self.get_logger().warn(f"[{robot_B.state.name}] don't have path, please check!")
                return robot_A.state.name

            if robotA_path_dis >= robotB_path_dis:
                result = robot_A.state.name
            else:
                result = robot_B.state.name

        if result == robot_A.state.name:
            self.get_logger().warn(
                f"[{robot_B.state.name}] will pause for conflicts handle (code: {code})!"
            )
        else:
            self.get_logger().warn(
                f"[{robot_A.state.name}] will pause for conflicts handle (code: {code})!"
            )
        return result

    # Hàm tính toán tích vô hướng của 2 vector
    def dot_product(self, v1, v2):
        return v1[0] * v2[0] + v1[1] * v2[1]

    # Hàm chiếu các đỉnh của hình chữ nhật lên một trục và trả về interval (min, max)
    def project_rectangle(self, rect: Rectangle, axis):
        projections = [self.dot_product(corner, axis) for corner in rect.corners]
        return (min(projections), max(projections))

    # Hàm kiểm tra sự giao nhau giữa 2 hình chữ nhật với phương pháp SAT
    def rectangles_intersect(self, rect1: Rectangle, rect2: Rectangle):
        for rect in [rect1, rect2]:
            for i in range(4):
                p1 = rect.corners[i]
                p2 = rect.corners[(i + 1) % 4]
                edge = (p2[0] - p1[0], p2[1] - p1[1])
                axis = self.perpendicular(edge)

                proj1 = self.project_rectangle(rect1, axis)
                proj2 = self.project_rectangle(rect2, axis)
                if proj1[1] < proj2[0] or proj2[1] < proj1[0]:
                    return False
        return True

    def perpendicular(self, v):
        return (-v[1], v[0])

    def calculate_midpoint(self, point1, point2):
        midpoint_x = (point1[0] + point2[0]) / 2
        midpoint_y = (point1[1] + point2[1]) / 2
        return (midpoint_x, midpoint_y)

    def calculate_rectangle(self, position: Location, width, height, front_extension):
        cos_yaw = math.cos(position.yaw)
        sin_yaw = math.sin(position.yaw)

        front_left = (
            position.x - (width / 2) * sin_yaw + (height / 2) * cos_yaw,
            position.y + (width / 2) * cos_yaw + (height / 2) * sin_yaw,
        )
        front_right = (
            position.x + (width / 2) * sin_yaw + (height / 2) * cos_yaw,
            position.y - (width / 2) * cos_yaw + (height / 2) * sin_yaw,
        )
        back_left = (
            position.x - (width / 2) * sin_yaw - (height / 2) * cos_yaw,
            position.y + (width / 2) * cos_yaw - (height / 2) * sin_yaw,
        )
        back_right = (
            position.x + (width / 2) * sin_yaw - (height / 2) * cos_yaw,
            position.y - (width / 2) * cos_yaw - (height / 2) * sin_yaw,
        )
        midpoint_front = self.calculate_midpoint(front_left, front_right)

        front_left_extension = (
            midpoint_front[0] - (width / 2) * sin_yaw + front_extension * cos_yaw,
            midpoint_front[1] + (width / 2) * cos_yaw + front_extension * sin_yaw,
        )
        front_right_extension = (
            midpoint_front[0] + (width / 2) * sin_yaw + front_extension * cos_yaw,
            midpoint_front[1] - (width / 2) * cos_yaw + front_extension * sin_yaw,
        )
        rectangle = Rectangle(front_left_extension, front_right_extension, back_left, back_right)
        return rectangle

    # Kiểm tra xem robot nào đang có xu hướng đâm vào robot kia
    def check_collision_direction(
        self, robot_name1: str, position1: Location, robot_name2: str, position2: Location
    ):
        # Tính toán vector hướng di chuyển của robot 1
        yaw_vector1 = (math.cos(position1.yaw), math.sin(position1.yaw))
        # Tính toán vector hướng di chuyển của robot 2
        yaw_vector2 = (math.cos(position2.yaw), math.sin(position2.yaw))

        # Tính khoảng cách giữa các robot
        distance_x = position2.x - position1.x
        distance_y = position2.y - position1.y

        # Kiểm tra hướng di chuyển của robot 1 có hướng về robot 2 không
        dot1 = self.dot_product(yaw_vector1, (distance_x, distance_y))
        dot2 = self.dot_product(yaw_vector2, (-distance_x, -distance_y))

        # Nếu dot_product là dương, robot có xu hướng di chuyển về phía robot kia
        if dot1 > 0 and dot2 > 0:
            self.get_logger().info(
                f"[{robot_name1}] và [{robot_name2}] đang có xu hướng đâm vào nhau"
            )
            return 3
        elif dot1 > 0:
            self.get_logger().info(f"[{robot_name1}] có xu hướng đâm vào [{robot_name2}]")
            return 1
        elif dot2 > 0:
            self.get_logger().info(f"[{robot_name2}] có xu hướng đâm vào [{robot_name1}]")
            return 2
        return 0

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

    def _conflict_handle_cb(self):
        robot_on_levels = {}
        for robot1Name, robot1State in self.robots.items():
            if robot1State.state is None:
                continue

            robot1Level = robot1State.state.location.level_name
            if self.debug and robot1Level != "":
                if robot1Level not in robot_on_levels:
                    robot_on_levels.update({robot1Level: [robot1State]})
                else:
                    robot_on_levels[robot1Level].append(robot1State)

            robot1Mode = robot1State.state.mode.mode
            if (
                robot1Mode == RobotMode.MODE_IDLE
                or robot1Mode == RobotMode.MODE_CHARGING
                or robot1Mode == RobotMode.MODE_EMERGENCY
                or robot1Mode == RobotMode.MODE_REQUEST_ERROR
            ):
                if len(robot1State.state.path) == 0:
                    if robot1State.wait_HID is not None:
                        self.robots[robot1State.wait_HID].wait_LID.remove(robot1Name)
                        robot1State.wait_HID = None
                        robot1State.last_mode_request = None
                        self.get_logger().info(
                            f"Robot [{robot1Name}] is not moving or pause will reset state!"
                        )

                    if len(robot1State.wait_LID) != 0:
                        for robot in robot1State.wait_LID:
                            self.mode_request(
                                fleet_name=self.robots[robot].fleet_name,
                                robot_name=robot,
                                mode=RobotMode.MODE_MOVING,
                            )
                            robot1State.wait_LID.remove(robot)
                            self.robots[robot].wait_HID = None
                            self.robots[robot].last_mode_request = None
                            self.get_logger().info(
                                f"Publish RESUME_ACTION for [{robot}] from conflicts handle!"
                            )
                continue

            for robot2Name, robot2State in self.robots.items():
                if robot1Name == robot2Name or robot2State.state is None:
                    continue

                robot2Mode = robot2State.state.mode.mode
                # Kiểm tra xem 2 robot này có cùng tầng không:
                if robot1Level == robot2State.state.location.level_name:
                    posA = robot1State.state.location
                    posB = robot2State.state.location
                    if len(robot1State.state.path) > 0 and len(robot2State.state.path) > 0:
                        rectA = self.calculate_rectangle(
                            posA,
                            self.width_conflict,
                            self.height_conflict,
                            self.front_extension,
                        )
                        rectB = self.calculate_rectangle(
                            posB,
                            self.width_conflict,
                            self.height_conflict,
                            self.front_extension,
                        )
                        # Kiểm tra 2 robot có xự xâm lấn vùng conflict hay không
                        if self.rectangles_intersect(rectA, rectB):
                            # Kiểm tra xem robot1 có dang di chuyển hay không:
                            if robot1Mode == RobotMode.MODE_MOVING:
                                if self.debug:
                                    self.get_logger().warn(
                                        f"Detect overlap zone conflict of [{robot1Name}]  with [{robot2Name}]!"
                                    )

                                if robot2Mode == RobotMode.MODE_MOVING:
                                    checker = self.check_collision_direction(
                                        robot1Name, posA, robot2Name, posB
                                    )
                                    prioHighRobot, prioLowRobot = [robot1State, robot2State]
                                    if checker == 0:
                                        continue
                                    elif checker == 1:
                                        prioHighRobot = robot2State
                                        prioLowRobot = robot1State
                                    elif checker == 2:
                                        prioHighRobot = robot1State
                                        prioLowRobot = robot2State
                                    elif checker == 3:
                                        # Robot nào có độ ưu tiên thấp hơn sẽ phải chuyển sang chế độ tạm dừng
                                        prioHID = self.check_priority(robot1State, robot2State)
                                        if prioHID == robot1Name:
                                            prioHighRobot = robot1State
                                            prioLowRobot = robot2State
                                        else:
                                            prioHighRobot = robot2State
                                            prioLowRobot = robot1State

                                    if (
                                        prioHighRobot.last_mode_request is None
                                        and prioLowRobot.last_mode_request is None
                                        and prioLowRobot.last_mode_request != RobotMode.MODE_PAUSED
                                    ):
                                        # Yêu cầu robot không được ưu tiên sẽ chuyển sang MODE_PAUSED
                                        self.mode_request(
                                            fleet_name=prioLowRobot.fleet_name,
                                            robot_name=prioLowRobot.state.name,
                                            mode=RobotMode.MODE_PAUSED,
                                        )
                                        prioLowRobot.last_mode_request = RobotMode.MODE_PAUSED
                                        prioLowRobot.wait_HID = prioHighRobot.state.name
                                        prioHighRobot.wait_LID.append(prioLowRobot.state.name)
                                        self.get_logger().warn(
                                            f"Publish PAUSED_ACTION for [{prioLowRobot.state.name}] (waiting [{prioHighRobot.state.name}])!"
                                        )

                                # Nếu robot2 dang ở chế độ tạm dừng bởi wait_HID khác thì robot1 cũng
                                # sẽ chuyển sang chế độ tạm dừng để tránh xung đột với wait_HID của robot2
                                elif (
                                    robot2Mode == RobotMode.MODE_PAUSED
                                    and robot2State.wait_HID is not None
                                    and robot2State.wait_HID != robot1Name
                                    and robot1State.last_mode_request != RobotMode.MODE_PAUSED
                                ):
                                    self.mode_request(
                                        fleet_name=robot1State.fleet_name,
                                        robot_name=robot1Name,
                                        mode=RobotMode.MODE_PAUSED,
                                    )
                                    robot1State.last_mode_request = RobotMode.MODE_PAUSED
                                    robot1State.wait_HID = robot2Name
                                    robot2State.wait_LID.append(robot1Name)
                                    self.get_logger().warn(
                                        f"Publish PAUSED_ACTION for [{robot1Name}] (waiting [{robot2Name}])!"
                                    )
                        # Không có sự xâm lấn, nếu robot1 đang chờ robot2 hãy giải phóng robot1
                        elif (
                            robot1Mode == RobotMode.MODE_PAUSED
                            and robot1State.wait_HID == robot2Name
                            and robot1State.last_mode_request is not None
                        ):
                            self.mode_request(
                                fleet_name=robot1State.fleet_name,
                                robot_name=robot1Name,
                                mode=RobotMode.MODE_MOVING,
                            )
                            robot2State.wait_LID.remove(robot1Name)
                            robot1State.last_mode_request = None
                            robot1State.wait_HID = None
                            self.get_logger().info(
                                f"Publish RESUME_ACTION for [{robot1Name}] from conflicts handle!"
                            )

                # Nếu robot khác tầng với nhau, hãy kiểm tra nếu robot1
                # đang tạm dừng để chờ robot2 hãy giải phóng robot1
                elif (
                    robot1Mode == RobotMode.MODE_PAUSED
                    and robot1State.wait_HID == robot2Name
                    and robot1State.last_mode_request is not None
                ):
                    self.mode_request(
                        fleet_name=robot1State.fleet_name,
                        robot_name=robot1Name,
                        mode=RobotMode.MODE_MOVING,
                    )
                    robot2State.wait_LID.remove(robot1Name)
                    robot1State.last_mode_request = None
                    robot1State.wait_HID = None
                    self.get_logger().info(
                        f"Publish RESUME_ACTION for [{robot1Name}] from conflicts handle!"
                    )
        if len(robot_on_levels) != 0:
            self.update_plot(robot_on_levels)

    def fleet_states_cb(self, msg: FleetState):
        robotsData = msg.robots
        for robot in robotsData:
            if robot.name in self.robots:
                self.robots[robot.name].state = robot


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
        "-c1",
        "--config_file_1",
        type=str,
        required=True,
        help="Path to the all config.yaml file",
    )
    parser.add_argument(
        "-c2",
        "--config_file_2",
        type=str,
        required=True,
        help="Path to the all config.yaml file",
    )
    args = parser.parse_args(args_without_ros[1:])
    print(f"Starting fleet conflicts handle...")

    configs = []
    with open(args.config_file_1, "r") as f:
        config = yaml.safe_load(f)
        configs.append(config)

    with open(args.config_file_2, "r") as f:
        config = yaml.safe_load(f)
        configs.append(config)

    fleet_conflicts_handle = FleetConflictsHandle(configs)
    # executor = MultiThreadedExecutor()
    # executor.add_node(fleet_conflicts_handle)
    # try:
    #     fleet_conflicts_handle.get_logger().info("Beginning client, shut down with CTRL-C")
    #     executor.spin()
    # except KeyboardInterrupt:
    #     fleet_conflicts_handle.get_logger().info("Keyboard interrupt, shutting down.\n")
    # fleet_conflicts_handle.destroy_node()
    # rclpy.shutdown()
    # ////////////////////////////////////

    rclpy.spin(fleet_conflicts_handle)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fleet_conflicts_handle.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
