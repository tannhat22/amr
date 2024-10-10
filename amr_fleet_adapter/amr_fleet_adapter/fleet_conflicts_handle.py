import sys
import argparse
import yaml
import math

import rclpy
from rclpy.node import Node

# from rclpy.qos import QoSProfile
# from rclpy.qos import QoSHistoryPolicy as History
# from rclpy.qos import QoSDurabilityPolicy as Durability
# from rclpy.qos import QoSReliabilityPolicy as Reliability
from rclpy.qos import qos_profile_system_default

from rmf_fleet_msgs.msg import FleetState, RobotMode, RobotState, ModeRequest


class State:
    def __init__(self, last_mode_request: RobotMode = None):
        self.last_mode_request = last_mode_request
        self.wait_HID = None
        self.wait_LID = []


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
        for robot_name, robot_config in self.config["robots"].items():
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
    # return ID of robot have higher priority
    def check_priority(self, A: int, B: int, robot_A: RobotState, robot_B: RobotState):
        if robot_A.battery_percent <= self.recharge_threshold:
            return A
        elif robot_B.battery_percent <= self.recharge_threshold:
            return B
        elif robot_A.battery_percent > robot_B.battery_percent:
            if len(robot_A.path):
                endPath_A = [robot_A.path[-1].x, robot_A.path[-1].y]
                if (
                    self.dist(endPath_A, [robot_B.location.x, robot_B.location.y])
                    >= self.conflicts_distance
                ):
                    return A
                else:
                    return B
            return B
        elif robot_A.battery_percent < robot_B.battery_percent:
            if len(robot_B.path):
                endPath_B = [robot_B.path[-1].x, robot_B.path[-1].y]
                if (
                    self.dist(endPath_B, [robot_A.location.x, robot_A.location.y])
                    >= self.conflicts_distance
                ):
                    return B
                else:
                    return A
            return A
        else:
            if len(robot_A.path) > len(robot_B.path):
                endPath_A = [robot_A.path[-1].x, robot_A.path[-1].y]
                if (
                    self.dist(endPath_A, [robot_B.location.x, robot_B.location.y])
                    >= self.conflicts_distance
                ):
                    return A
                else:
                    return B
            else:
                if len(robot_B.path):
                    endPath_B = [robot_B.path[-1].x, robot_B.path[-1].y]
                    if (
                        self.dist(endPath_B, [robot_A.location.x, robot_A.location.y])
                        >= self.conflicts_distance
                    ):
                        return B
                    else:
                        return A
                return A

    def fleet_states_cb(self, msg: FleetState):
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
                            f"Robot: `{robot}` will resume from conflicts"
                        )

                    self.robots[robot1Name].wait_LID = []

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
                                if (
                                    self.check_priority(
                                        A=i,
                                        B=j,
                                        robot_A=dataRobot[i],
                                        robot_B=dataRobot[j],
                                    )
                                    == i
                                ):
                                    prioHID = i
                                    prioLID = j
                                else:
                                    prioHID = j
                                    prioLID = i

                                # Yêu cầu robot không được ưu tiên sẽ chuyển sang MODE_PAUSED
                                if (
                                    self.robots[
                                        dataRobot[prioLID].name
                                    ].last_mode_request
                                    != RobotMode.MODE_PAUSED
                                ):
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
                                    self.get_logger().info(
                                        f"Robot `{dataRobot[prioLID].name}` will waiting"
                                        f" `{dataRobot[prioHID].name}` for conflicts"
                                    )

                            # Nếu robot2 đang docking thì tạm dừng robot1
                            elif robot2Mode == RobotMode.MODE_DOCKING:
                                if (
                                    self.robots[robot1Name].last_mode_request
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
                                    self.get_logger().info(
                                        f"Robot `{robot1Name}` will waiting"
                                        f" `{robot2Name}` for conflicts"
                                    )

                            # Nếu robot2 dang ở chế độ tạm dừng bởi wait_HID khác thì robot1 cũng
                            # sẽ chuyển sang chế độ tạm dừng để tránh xung đột với wait_HID của robot2
                            elif robot2Mode == RobotMode.MODE_PAUSED:
                                if (
                                    self.robots[robot2Name].wait_HID is not None
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
                                    self.get_logger().info(
                                        f"Robot `{robot1Name}` will waiting"
                                        f" `{robot2Name}` for conflicts"
                                    )
                        # Kiểm tra robot1 đang docking hay không
                        elif robot1Mode == RobotMode.MODE_DOCKING:
                            if robot2Mode == RobotMode.MODE_MOVING:
                                if (
                                    self.robots[robot2Name].last_mode_request
                                    != RobotMode.MODE_PAUSED
                                ):
                                    self.mode_request(
                                        fleet_name=fleetName,
                                        robot_name=robot2Name,
                                        mode=RobotMode.MODE_PAUSED,
                                    )
                                    self.robots[robot2Name].last_mode_request = (
                                        RobotMode.MODE_PAUSED
                                    )
                                    self.robots[robot2Name].wait_HID = robot1Name
                                    self.robots[robot1Name].wait_LID.append(robot2Name)
                                    self.get_logger().info(
                                        f"Robot `{robot2Name}` will waiting"
                                        f" `{robot1Name}` for conflicts"
                                    )
                    else:
                        # Khi khoảng cách đã vượt ngoài ngưỡng conflict, hãy kiểm tra
                        # nếu robot1 đang tạm dừng để chờ robot2 hãy giải phóng robot1
                        if (
                            self.robots[robot1Name].last_mode_request
                            == RobotMode.MODE_PAUSED
                            and self.robots[robot1Name].wait_HID == robot2Name
                        ):
                            self.mode_request(
                                fleet_name=fleetName,
                                robot_name=robot1Name,
                                mode=RobotMode.MODE_MOVING,
                            )
                            self.robots[robot1Name].last_mode_request = None
                            self.robots[robot1Name].wait_HID = None
                            self.robots[robot2Name].wait_LID.remove(robot1Name)
                            self.get_logger().info(
                                f"Robot `{robot1Name}` will resume from conflicts"
                            )

                else:
                    if (
                        self.robots[robot1Name].last_mode_request
                        == RobotMode.MODE_PAUSED
                        and self.robots[robot1Name].wait_HID == robot2Name
                    ):
                        self.mode_request(
                            fleet_name=fleetName,
                            robot_name=robot1Name,
                            mode=RobotMode.MODE_MOVING,
                        )
                        self.robots[robot1Name].last_mode_request = None
                        self.robots[robot1Name].wait_HID = None
                        self.robots[robot2Name].wait_LID.remove(robot1Name)
                        self.get_logger().info(
                            f"Robot: `{robot1Name}` will resume from conflicts"
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
