import sys
import argparse
import yaml
import math

# import threading
import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node

# from rclpy.executors import MultiThreadedExecutor
# from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from enum import IntEnum
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rmf_fleet_msgs.msg import FleetState, RobotMode, RobotState, ModeRequest, Location


class State:
    def __init__(
        self,
        fleet_name: str = "",
        vicinity: float = 0.0,
        state: RobotState = None,
        # last_mode_request: RobotMode = None,
    ):
        self.fleet_name = fleet_name
        self.vicinity = vicinity
        self.state = state
        # self.last_mode_request = last_mode_request
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

    def __init__(self, configs):
        super().__init__(f"fleet_conflicts_handle")

        # Callback groups:
        # robot_cb_group = ReentrantCallbackGroup()

        # Params:
        self.declare_parameter("update_frequency", 5.0)
        self.declare_parameter("look_ahead_distance", 3.0)
        self.declare_parameter("debug", True)

        self.update_frequency = self.get_parameter("update_frequency").value
        self.look_ahead_distance = self.get_parameter("look_ahead_distance").value
        self.debug = self.get_parameter("debug").value

        self.get_logger().info(f"update_frequency: {self.update_frequency}")
        self.get_logger().info(f"look_ahead_distance: {self.look_ahead_distance}")
        if self.debug:
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

            fleet_name = config["rmf_fleet"]["name"]
            vicinity = config["rmf_fleet"]["profile"]["vicinity"]
            for robot_name, robot_config in config["rmf_fleet"]["robots"].items():
                self.robots[robot_name] = State(fleet_name=fleet_name, vicinity=vicinity)
            self.robots_length = len(self.robots)
        assert self.robots_length > 0

        self.recharge_threshold = config["rmf_fleet"]["recharge_threshold"]
        update_period = 1.0 / self.update_frequency

        # Threading variables
        # self._lock = threading.Lock()

        request_qos = QoSProfile(
            depth=10,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL,
        )

        self.mode_request_pub = self.create_publisher(
            ModeRequest,
            "action_execution_notice",
            qos_profile=request_qos,
        )

        self.create_subscription(FleetState, "fleet_states", self.fleet_states_cb, 10)

        self.create_timer(update_period, self._conflict_handle_cb)

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

    def point_to_segment_path_distance(
        self, position: Location, destination: Location, check_point: Location
    ):
        px = destination.x - position.x
        py = destination.y - position.y
        norm = px * px + py * py

        if norm == 0:
            return ((check_point.x - position.x) ** 2 + (check_point.y - position.y) ** 2) ** 0.5

        u = ((check_point.x - position.x) * px + (check_point.y - position.y) * py) / norm
        u = max(0, min(1, u))

        # Nếu vị trí đến vật cản nằm phía sau vị trí robot hiện tại so với hướng đường đi hiện tại thì bỏ qua vật cản
        if u == 0:
            return 100.0

        closest_x = position.x + u * px
        closest_y = position.y + u * py

        return ((check_point.x - closest_x) ** 2 + (check_point.y - closest_y) ** 2) ** 0.5

    def get_look_ahead_point(
        self, position: Location, destination: Location, look_ahead_distance: float
    ):
        # Tính vector hướng
        dx, dy = destination.x - position.x, destination.y - position.y
        length = np.sqrt(dx**2 + dy**2)

        if length == 0:  # Nếu robot đã ở đích
            return position

        # Tính điểm giới hạn khoảng nhìn trước
        scale = min(look_ahead_distance / length, 1)
        look_ahead_point = Location()
        look_ahead_point.x = position.x + scale * dx
        look_ahead_point.y = position.y + scale * dy
        return look_ahead_point

    def calculate_midpoint(self, point1, point2):
        midpoint_x = (point1[0] + point2[0]) / 2
        midpoint_y = (point1[1] + point2[1]) / 2
        return (midpoint_x, midpoint_y)

    # Kiểm tra robot có va chạm với robot khác trên đường di chuyển trong phạm vi look ahead không
    def check_collision_direction(
        self,
        robot1_state: State,
        robot1_look_ahead_point: Location,
        robot2_state: State,
    ):
        pos1 = robot1_state.state.location
        pos2 = robot2_state.state.location

        # Kiểm tra khoảng cách từ pos2 đến đoạn thẳng giữa pos1 và look_ahead_point
        dist = self.point_to_segment_path_distance(pos1, robot1_look_ahead_point, pos2)

        if dist < robot1_state.vicinity + robot2_state.vicinity:
            return True

        return False

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
        if self.debug:
            for level, ax in self.levels_ax.items():
                ax.clear()
                ax.set_title(f"LEVELS: [{level}]")
                ax.set_xlabel("X")
                ax.set_ylabel("Y")
                ax.set_xlim(50, 80)
                ax.set_ylim(-10, -40)
                ax.set_aspect("equal", adjustable="box")
                ax.grid(True)

        color_count = 0
        for robot1Name, robot1State in self.robots.items():
            color_count += 1
            if robot1State.state is None:
                continue

            robot1Level = robot1State.state.location.level_name

            # Vẽ robot
            if self.debug:
                x = robot1State.state.location.x
                y = robot1State.state.location.y
                ax = self.levels_ax[robot1Level]
                ax.scatter(x, y, label=robot1Name, c=f"C{color_count}")
                circle = plt.Circle(
                    (x, y),
                    robot1State.vicinity,
                    color=f"C{color_count}",
                    alpha=0.3,
                )
                ax.add_artist(circle)

            if len(robot1State.state.path) > 0:
                pos1 = robot1State.state.location
                dest1 = robot1State.state.path[-1]
                look_ahead_point = self.get_look_ahead_point(pos1, dest1, self.look_ahead_distance)

                if self.debug:
                    # Vẽ đoạn thẳng từ vị trí robot hiện tại đến đích của nó:
                    ax.plot([pos1.x, dest1.x], [pos1.y, dest1.y], "--", color=f"C{color_count}")
                    ax.arrow(
                        pos1.x,
                        pos1.y,
                        look_ahead_point.x - pos1.x,
                        look_ahead_point.y - pos1.y,
                        head_width=0.5,
                        head_length=0.7,
                        length_includes_head=True,
                        fc="red",
                        ec="red",
                    )
                    # Vẽ vị trí đích đến và chú thích tên
                    ax.plot(
                        dest1.x,
                        dest1.y,
                        "x",
                        color=f"C{color_count}",
                        markersize=10,
                        label=f"Destination [{robot1Name}]",
                    )

                detect_obtacles = False
                for robot2Name, robot2State in self.robots.items():
                    if robot1Name == robot2Name or robot2State.state is None:
                        continue

                    # Kiểm tra xem 2 robot này có cùng tầng không:
                    if robot1Level == robot2State.state.location.level_name:
                        if self.check_collision_direction(
                            robot1State, look_ahead_point, robot2State
                        ):
                            if (
                                robot1Name == robot2State.wait_HID
                                and robot1State.state.avoid_obstacles
                            ):
                                self.get_logger().warn(
                                    f"Robot [{robot1Name}] allowed to move because robot [{robot2Name}] is waiting for it, and it can avoid_obstacles!"
                                )                                
                                continue

                            detect_obtacles = True
                            # if robot1State.last_mode_request is None:
                            if robot1State.state.mode.mode == RobotMode.MODE_MOVING:
                                self.mode_request(
                                    fleet_name=robot1State.fleet_name,
                                    robot_name=robot1Name,
                                    mode=RobotMode.MODE_WAITING,
                                )
                                # robot1State.last_mode_request = RobotMode.MODE_WAITING
                                robot1State.wait_HID = robot2Name
                                self.get_logger().warn(
                                    f"Robot[{robot1Name}] need wait for detect collision!"
                                )
                            break

                # if robot1State.last_mode_request == RobotMode.MODE_WAITING and not detect_obtacles:
                if not detect_obtacles:
                    # robot1State.last_mode_request = None
                    robot1State.wait_HID = None
                    if robot1State.state.mode.mode == RobotMode.MODE_WAITING:
                        self.mode_request(
                            fleet_name=robot1State.fleet_name,
                            robot_name=robot1Name,
                            mode=RobotMode.MODE_MOVING,
                        )
                        self.get_logger().warn(
                            f"Robot[{robot1Name}] resume moving because obstacles is clearing!"
                        )
                # else:
                #     break
            else:
                # robot1State.last_mode_request = None
                robot1State.wait_HID = None

        if self.debug:
            for level, ax in self.levels_ax.items():
                ax.legend()
            self.fig.canvas.draw()  # Cập nhật biểu đồ
            self.fig.canvas.flush_events()  # Đảm bảo sự kiện được thực thi

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

    if args.config_file_1 != "":
        with open(args.config_file_1, "r") as f:
            config = yaml.safe_load(f)
            configs.append(config)

    if args.config_file_2 != "":
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
