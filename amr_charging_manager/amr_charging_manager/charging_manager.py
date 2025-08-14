import sys
import argparse
import yaml
import json
import time
import rclpy

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    qos_profile_system_default,
)
from std_msgs.msg import String
from rmf_fleet_msgs.msg import Location, ChargingAssignment, ChargingAssignments


class RobotState:
    def __init__(
        self,
        status: str = None,
        battery: float = 1.2,
        location: Location = None,
    ):
        self.status = status
        self.battery = battery
        self.location = location
        self.need_charging: float = None


class Waypoint:
    def __init__(self, name: str, is_charger: bool = False):
        self.name = name
        self.is_charger = is_charger
        self.assigned_robot: str = None
        self.assigned_since: float = None


class FleetChargingConfigure:
    def __init__(
        self,
        robots: dict[str, RobotState] = {},
        use_recharge_threshold: bool = False,
        recharge_threshold: float = 0.5,
        charger: list[Waypoint] = [],
        parking: list[Waypoint] = [],
    ):
        self.robots = robots
        self.use_recharge_threshold = use_recharge_threshold
        self.recharge_threshold = recharge_threshold
        self.charger = charger
        self.parking = parking


class ChargingManager(Node):
    fleets: dict[str, FleetChargingConfigure]

    def __init__(self, config: dict):
        super().__init__(f"charging_manager")

        # Params:
        self.declare_parameter("update_frequency", 1.0)
        self.declare_parameter("min_charge_time", 60.0)
        self.declare_parameter("debug", True)

        self.update_frequency = self.get_parameter("update_frequency").value
        self.min_charge_time = self.get_parameter("min_charge_time").value
        self.debug = self.get_parameter("debug").value

        self.get_logger().info(f"update_frequency: {self.update_frequency}")
        self.get_logger().info(f"min_charge_time: {self.min_charge_time}")

        # if self.debug:
        #     pass

        self.fleets = {}
        self.fleets_assignments = {}
        fleets_config = config["fleets"]
        for name, fleet in fleets_config.items():
            robots = fleet["robots"]
            use_recharge_threshold = fleet["use_recharge_threshold"]
            recharge_threshold = fleet["recharge_threshold"]
            charger = fleet["charger"]
            parking = fleet["parking"]

            assert len(robots) > 0, "robots is empty, please check!"

            assert (
                len(charger) > 0
            ), "fleet configure must have at least 1 charging station!"

            if use_recharge_threshold:
                assert len(charger) == len(robots) and len(charger) == len(
                    parking
                ), "use_recharge_threshold is True but unequal number of robots, positions charger and parking!"
            else:
                assert len(robots) <= (
                    len(charger) + len(parking)
                ), "use_recharge_threshold is False but unequal number of robots and sum of positions charger, parking"

            robotDict = {}
            for robot in robots:
                robotDict.update({robot: RobotState()})

            cwp = []
            for c in charger:
                cwp.append(Waypoint(c, True))

            pwp = []
            for p in parking:
                pwp.append(Waypoint(p, False))

            self.fleets.update(
                {
                    name: FleetChargingConfigure(
                        robotDict,
                        use_recharge_threshold,
                        recharge_threshold,
                        cwp,
                        pwp,
                    )
                }
            )

        transient_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2 * len(self.fleets),
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.charging_assignments_pub = self.create_publisher(
            ChargingAssignments, "charging_assignments", qos_profile=transient_qos
        )

        # Khởi tạo dữ liệu phân bổ trạm sạc và đỗ
        self._charging_handle_cb()

        self.create_subscription(
            String,
            "/fleet_state_update",
            self.fleet_state_update_cb,
            qos_profile=qos_profile_system_default,
        )

        update_period = 1.0 / self.update_frequency
        self.create_timer(update_period, self._charging_handle_cb)

    def publish_assignments(self, fleet: str, assignments: dict[str, Waypoint]):
        msg = ChargingAssignments()
        msg.fleet_name = fleet
        for robot, waypoint in assignments.items():
            assignment = ChargingAssignment()
            assignment.robot_name = robot
            assignment.waypoint_name = waypoint.name

            if waypoint.is_charger:
                assignment.mode = ChargingAssignment.MODE_CHARGE
            else:
                assignment.mode = ChargingAssignment.MODE_WAIT
            msg.assignments.append(assignment)

        self.charging_assignments_pub.publish(msg)

    def _charging_handle_cb(self):
        current_time = time.time()
        for fleet_name, fleet_charging in self.fleets.items():
            assignments: dict[str, Waypoint]
            assignments = {}
            if fleet_charging.use_recharge_threshold:
                count = 0
                for name, robot in fleet_charging.robots.items():
                    if robot.battery >= fleet_charging.recharge_threshold:
                        if (
                            robot.need_charging is None
                            or robot.battery >= robot.need_charging
                        ):
                            assignments[name] = fleet_charging.parking[count]
                            robot.need_charging = None
                        else:
                            assignments[name] = fleet_charging.charger[count]
                            robot.need_charging = 0.95
                    else:
                        assignments[name] = fleet_charging.charger[count]
                        robot.need_charging = 0.95
                    count += 1
            else:
                robot_battery_sorted = sorted(
                    fleet_charging.robots,
                    key=lambda k: fleet_charging.robots[k].battery,
                )

                for robot_name in robot_battery_sorted:
                    # Xử lý vị trí sạc cho các robot được ưu tiên và ưu tiên tiếp tục sạc tại vị trí trước đó
                    assigned_charge = False
                    charger_id_available = None
                    robot = fleet_charging.robots[robot_name]
                    for id, charger in enumerate(fleet_charging.charger):
                        assigned_robot_name = charger.assigned_robot
                        # Trạm đang rảnh
                        if assigned_robot_name is None:
                            # if robot.status is None:
                            #     charger.assigned_since = 0.0
                            # else:
                            #     charger.assigned_since = current_time
                            charger.assigned_since = 0.0
                            charger.assigned_robot = robot_name
                            assignments[robot_name] = charger
                            assigned_charge = True
                            break
                        # Trạm đã được gán cho robot này
                        elif assigned_robot_name == robot_name:
                            assignments[robot_name] = charger
                            assigned_charge = True
                            break
                        # Trạm đã được gán cho robot khác thì sẽ kiểm tra có thể xin quyền sử dụng trạm này được không
                        elif charger_id_available is None:
                            other_robot = fleet_charging.robots[assigned_robot_name]
                            time_charging = current_time - charger.assigned_since
                            if robot.status is not None:
                                if other_robot.status is None:
                                    # Nếu robot khác không hoạt động thì có thể đổi quyền sử dụng trạm này
                                    charger_id_available = id
                                elif (
                                    robot_battery_sorted.index(assigned_robot_name)
                                    >= len(fleet_charging.charger)
                                    and other_robot.battery > robot.battery
                                    and other_robot.status != "charging"
                                    and time_charging >= self.min_charge_time * 60.0
                                ):
                                    # Nếu robot này pin thấp hơn robot khác và đã sạc đủ thời gian tối thiểu thì có thể đổi quyền sử dụng trạm này
                                    charger_id_available = id

                    # Nếu chưa được gán trạm sạc thì kiểm tra lần nữa có trạm này sẵn sàng đổi quyền sử dụng không
                    if not assigned_charge and charger_id_available is not None:
                        # Cho robot pin thấp hơn vào sạc
                        fleet_charging.charger[charger_id_available].assigned_robot = (
                            robot_name
                        )
                        fleet_charging.charger[charger_id_available].assigned_since = (
                            current_time
                        )
                        assignments[robot_name] = fleet_charging.charger[
                            charger_id_available
                        ]
                        assigned_charge = True

                    # Xử lý vị trí đỗ cho các robot không được sạc có ưu tiên tiếp tục đỗ tại vị trí trước đó
                    assigned_parking = False
                    parking_id_available = None
                    for id, parking in enumerate(fleet_charging.parking):
                        assigned_robot_name = parking.assigned_robot
                        if assigned_robot_name == robot_name:
                            if assigned_charge:
                                parking.assigned_robot = None
                            else:
                                assignments[robot_name] = parking
                                assigned_parking = True
                            break
                        elif (
                            assigned_robot_name is None and parking_id_available is None
                        ):
                            parking_id_available = id

                    if (
                        not assigned_charge
                        and not assigned_parking
                        and parking_id_available is not None
                    ):
                        fleet_charging.parking[parking_id_available].assigned_robot = (
                            robot_name
                        )
                        assignments[robot_name] = fleet_charging.parking[
                            parking_id_available
                        ]

            need_update_assignments = False
            last_fleet_assignments = self.fleets_assignments.get(fleet_name, None)
            if last_fleet_assignments is not None:
                for robot, waypoint in assignments.items():
                    if self.fleets_assignments[fleet_name][robot].name != waypoint.name:
                        need_update_assignments = True
                        break
            else:
                need_update_assignments = True

            # Chỉ cập nhật và publish dữ liệu khi có sự thay đổi assignments
            if need_update_assignments:
                self.fleets_assignments[fleet_name] = assignments
                self.publish_assignments(fleet_name, assignments)
                if self.debug:
                    self.get_logger().info(
                        f"New charging assignments for fleet name: [{fleet_name}]"
                    )
                    for robot, waypoint in assignments.items():
                        self.get_logger().info(
                            f"    [{robot}] ({round(fleet_charging.robots[robot].battery * 100)}%): {waypoint.name}"
                        )

                    self.get_logger().info(
                        "-------------------------------------------------------------"
                    )

    def fleet_state_update_cb(self, msg: String):
        fleetStateUpdate = json.loads(msg.data)
        fleetState = fleetStateUpdate["data"]

        fleetContext = self.fleets.get(fleetState["name"], None)
        if fleetContext is not None:
            for name, robot in fleetState["robots"].items():
                if name in fleetContext.robots:
                    fleetContext.robots[name].status = robot["status"]
                    fleetContext.robots[name].battery = robot["battery"]
                    fleetContext.robots[name].location = robot["location"]


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def main(argv=sys.argv):
    # Init rclpy
    rclpy.init(args=argv)
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="charging_manager",
        description="Configure and spin up the charging manager",
    )
    parser.add_argument(
        "-c",
        "--config_file",
        type=str,
        required=True,
        help="Path to the all config.yaml file",
    )
    args = parser.parse_args(args_without_ros[1:])
    print(f"Starting charging manager...")

    config = {}

    if args.config_file != "":
        with open(args.config_file, "r") as f:
            config = yaml.safe_load(f)

    charging_manager = ChargingManager(config)
    rclpy.spin(charging_manager)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    charging_manager.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
