import sys
import argparse
import yaml
import math

import rclpy
from enum import IntEnum
from rclpy.node import Node

# from rclpy.qos import QoSProfile
# from rclpy.qos import QoSHistoryPolicy as History
# from rclpy.qos import QoSDurabilityPolicy as Durability
# from rclpy.qos import QoSReliabilityPolicy as Reliability
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
        self.declare_parameter("conflicts_distance", 3.0)
        self.declare_parameter("debug", True)

        self.conflicts_distance = self.get_parameter("conflicts_distance").value
        self.debug = self.get_parameter("debug").value

        self.robots = {}
        for robot_name, robot_config in self.config["rmf_fleet"]["robots"].items():
            self.robots[robot_name] = State()
        assert len(self.robots) > 0

        self.recharge_threshold = self.config["rmf_fleet"]["recharge_threshold"]
        self.current_cmd_id = ""

        # transient_qos = QoSProfile(
        #     history=History.KEEP_LAST,
        #     depth=1,
        #     reliability=Reliability.RELIABLE,
        #     durability=Durability.TRANSIENT_LOCAL)

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

    def dist(self, A, B):
        """Euclidian distance between A(x,y) and B(x,y)"""
        assert len(A) > 1
        assert len(B) > 1
        return math.sqrt((A[0] - B[0]) ** 2 + (A[1] - B[1]) ** 2)

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
        posCurrent_A = [robot_A.location.x, robot_A.location.y]
        posCurrent_B = [robot_B.location.x, robot_B.location.y]

        if len(robot_A.path) > 0:
            destA = [robot_A.path[-1].x, robot_A.path[-1].y]
        if len(robot_B.path) > 0:
            destB = [robot_B.path[-1].x, robot_B.path[-1].y]

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
            result = [A, B]
        elif robot_B.battery_percent / robot_A.battery_percent > 1.1:
            code = PriotityCode.BATTERY_PRIORITY
            result = [B, A]
        else:
            robotA_path_dis = self.calc_path_distance(robot_A.location, robot_A.path)
            robotB_path_dis = self.calc_path_distance(robot_B.location, robot_B.path)

            code = PriotityCode.PATH_DISTANCE_PRIORITY
            if robotA_path_dis >= robotB_path_dis:
                result = [A, B]
            else:
                result = [B, A]

        if result[0] == A:
            self.get_logger().info(
                f"[{robot_B.name}] will pause for conflicts handle (code: {code})!"
            )
        else:
            self.get_logger().info(
                f"[{robot_A.name}] will pause for conflicts handle (code: {code})!"
            )
        return result

    def calc_path_distance(self, position: Location, path: list[Location]):
        a = len(path)
        posCurrent = [position.x, position.y]
        if a == 0:
            return None
        else:
            pointDest = [path[0].x, path[0].y]
            dist = self.dist(posCurrent, pointDest)
            for i in range(a - 1):
                pointA = [path[i].x, path[i].y]
                pointB = [path[i + 1].x, path[i + 1].y]
                dist += self.dist(pointA, pointB)
            return dist

    def fleet_states_cb(self, msg: FleetState):
        dataRobot: list[RobotState]

        fleetName = msg.name
        dataRobot = msg.robots
        dataLen = len(dataRobot)
        for i in range(dataLen):
            robot1Name = dataRobot[i].name
            robot1Mode = dataRobot[i].mode.mode
            if (
                robot1Mode == RobotMode.MODE_IDLE
                or robot1Mode == RobotMode.MODE_CHARGING
                or robot1Mode == RobotMode.MODE_EMERGENCY
                or robot1Mode == RobotMode.MODE_REQUEST_ERROR
            ):
                if self.robots[robot1Name].wait_HID is not None:
                    self.robots[robot1Name].wait_HID = None
                    self.robots[robot1Name].last_mode_request = None
                    self.robots[self.robots[robot1Name].wait_HID].wait_LID.remove(
                        robot1Name
                    )

                if len(self.robots[robot1Name].wait_LID) != 0:
                    for robot in self.robots[robot1Name].wait_LID:
                        self.mode_request(
                            fleet_name=fleetName,
                            robot_name=robot,
                            mode=RobotMode.MODE_MOVING,
                        )
                        self.robots[robot].wait_HID = None
                        self.robots[robot].last_mode_request = None
                        self.get_logger().info(
                            f"Publish RESUME_ACTION for [{robot}] from conflicts handle!"
                        )
                        self.robots[robot1Name].wait_LID.remove(robot)
                continue

            for j in range(dataLen):
                if i == j:
                    continue

                robot2Name = dataRobot[j].name
                robot2Mode = dataRobot[j].mode.mode
                # Kiểm tra xem 2 robot này có cùng tầng không:
                if dataRobot[i].location.level_name == dataRobot[j].location.level_name:
                    posA = [dataRobot[i].location.x, dataRobot[i].location.y]
                    posB = [dataRobot[j].location.x, dataRobot[j].location.y]
                    # Kiểm tra khoảng cách giữa 2 robot nhỏ hơn conflicts_distance sẽ có thể gây xung đột
                    if self.dist(posA, posB) <= self.conflicts_distance:
                        # Kiểm tra xem robot1 có dang di chuyển hay không:
                        if robot1Mode == RobotMode.MODE_MOVING:
                            # Nếu robot2 cũng đang di chuyển thì phải kiểm tra mức độ ưu tiên,
                            # robot nào có độ ưu tiên thấp hơn sẽ phải chuyển sang chế độ tạm dừng
                            if robot2Mode == RobotMode.MODE_MOVING:
                                # Lựa chọn robot nào sẽ có độ ưu tiên cao hơn
                                prioHID, prioLID = self.check_priority(
                                    i, j, dataRobot[i], dataRobot[j]
                                )
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
                                if (
                                    dataRobot[prioLID].name
                                    not in self.robots[dataRobot[prioHID].name].wait_LID
                                ):
                                    self.robots[
                                        dataRobot[prioHID].name
                                    ].wait_LID.append(dataRobot[prioLID].name)
                                self.get_logger().info(
                                    f"Publish PAUSED_ACTION for [{dataRobot[prioLID].name}] (waiting [{dataRobot[prioHID].name}])!"
                                )

                            # Nếu robot2 đang docking thì tạm dừng robot1
                            elif robot2Mode == RobotMode.MODE_DOCKING:
                                self.mode_request(
                                    fleet_name=fleetName,
                                    robot_name=robot1Name,
                                    mode=RobotMode.MODE_PAUSED,
                                )
                                self.robots[robot1Name].last_mode_request = (
                                    RobotMode.MODE_PAUSED
                                )
                                self.robots[robot1Name].wait_HID = robot2Name
                                if robot1Name not in self.robots[robot2Name].wait_LID:
                                    self.robots[robot2Name].wait_LID.append(robot1Name)
                                self.get_logger().info(
                                    f"Publish PAUSED_ACTION for [{robot1Name}] (waiting [{robot2Name}])!"
                                )
                            # Nếu robot2 dang ở chế độ tạm dừng bởi wait_HID khác thì robot1 cũng
                            # sẽ chuyển sang chế độ tạm dừng để tránh xung đột với wait_HID của robot2
                            elif (
                                robot2Mode == RobotMode.MODE_PAUSED
                                and self.robots[robot2Name].wait_HID is not None
                                and self.robots[robot2Name].wait_HID != robot1Name
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
                                if robot1Name not in self.robots[robot2Name].wait_LID:
                                    self.robots[robot2Name].wait_LID.append(robot1Name)
                                self.get_logger().info(
                                    f"Publish PAUSED_ACTION for [{robot1Name}] (waiting [{robot2Name}])!"
                                )
                    else:
                        # Khi khoảng cách đã vượt ngoài ngưỡng conflict, hãy kiểm tra
                        # nếu robot1 đang chờ robot2 hãy giải phóng robot1
                        if (
                            robot1Mode == RobotMode.MODE_PAUSED
                            and self.robots[robot1Name].wait_HID == robot2Name
                        ):
                            self.mode_request(
                                fleet_name=fleetName,
                                robot_name=robot1Name,
                                mode=RobotMode.MODE_MOVING,
                            )
                            self.robots[robot1Name].last_mode_request = None
                            self.robots[robot1Name].wait_HID = None
                            if robot1Name in self.robots[robot2Name].wait_LID:
                                self.robots[robot2Name].wait_LID.remove(robot1Name)
                            self.get_logger().info(
                                f"Publish RESUME_ACTION for [{robot1Name}] from conflicts handle!"
                            )

                # Nếu robot khác tầng với nhau, hãy kiểm tra nếu robot1
                # đang tạm dừng để chờ robot2 hãy giải phóng robot1
                elif (
                    robot1Mode == RobotMode.MODE_PAUSED
                    and self.robots[robot1Name].wait_HID == robot2Name
                ):
                    self.mode_request(
                        fleet_name=fleetName,
                        robot_name=robot1Name,
                        mode=RobotMode.MODE_MOVING,
                    )
                    self.robots[robot1Name].last_mode_request = None
                    self.robots[robot1Name].wait_HID = None
                    if robot1Name in self.robots[robot2Name].wait_LID:
                        self.robots[robot2Name].wait_LID.remove(robot1Name)
                    self.get_logger().info(
                        f"Publish RESUME_ACTION for [{robot1Name}] from conflicts handle!"
                    )


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def main(argv=sys.argv):
    # Init rclpy and adapter
    rclpy.init(args=argv)
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="fleet_adapter", description="Configure and spin up the fleet adapter"
    )
    parser.add_argument(
        "-c",
        "--config_file",
        type=str,
        required=True,
        help="Path to the config.yaml file",
    )
    args = parser.parse_args(args_without_ros[1:])
    print(f"Starting fleet manager...")

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
