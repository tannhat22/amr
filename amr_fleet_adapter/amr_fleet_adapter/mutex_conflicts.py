# import threading
import json
import rclpy
import rclpy.wait_for_message
from rclpy.node import Node

# from rclpy.executors import MultiThreadedExecutor
# from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy as Reliability
from std_msgs.msg import String
from builtin_interfaces.msg import Time
from rmf_fleet_msgs.msg import (
    MutexGroupAssignment,
    MutexGroupManualRelease,
    MutexGroupRequest,
    MutexGroupStates,
)

UNCLAIMED = 18446744073709551615


class RobotState:
    def __init__(
        self,
        name: str = "",
        task_id: str = "",
        unix_millis_time: str = "",
        mutex_locked: list[str] = [],
        mutex_requesting: list[str] = [],
    ):
        self.name = name
        self.task_id = task_id
        self.unix_millis_time = unix_millis_time
        self.mutex_locked = mutex_locked
        self.mutex_requesting = mutex_requesting
        self.mutex_conflict = False


class MutexRequest:
    def __init__(
        self,
        claimant: str = "",
        last_request: MutexGroupRequest = None,
        last_time_request: float = 0.0,
    ):
        self.claimant = claimant
        self.last_request = last_request
        self.last_time_request = last_time_request


class MutexState:
    def __init__(
        self,
        claimant: int = UNCLAIMED,
        claim_time: Time = None,
        # robot_locked: str = "",
        # robot_requesting: list[str] = [],
    ):
        self.claimant = claimant
        self.claim_time = claim_time
        # self.robot_locked = robot_locked
        # self.robot_requesting = robot_requesting


class MutexConflicts(Node):
    # mutex_requests: dict[str, MutexRequest]
    robot_states: dict[str, RobotState]
    mutex_states: dict[str, MutexState]

    def __init__(self):
        super().__init__(f"mutex_conflicts_temporary")

        # Callback groups:
        # robot_cb_group = ReentrantCallbackGroup()

        # Params:
        self.declare_parameter("update_frequency", 5.0)
        self.declare_parameter("debug", True)

        self.update_frequency = self.get_parameter("update_frequency").value
        self.debug = self.get_parameter("debug").value

        self.get_logger().info(f"update_frequency: {self.update_frequency}")

        update_period = 1.0 / self.update_frequency

        # Threading variables
        # self._lock = threading.Lock()

        # Variables:
        self.robot_states = {}
        self.mutex_states = {}

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=10,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL,
        )

        sub_qos = QoSProfile(
            depth=10,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL,
        )

        # Publishers:
        self.mutex_group_req_pub = self.create_publisher(
            MutexGroupRequest,
            "/mutex_group_request",
            qos_profile=transient_qos,
        )

        # Subcribers:
        self.create_subscription(
            MutexGroupManualRelease,
            "/mutex_group_manual_release",
            self.mutex_group_manual_release_cb,
            qos_profile=sub_qos,
        )

        self.create_subscription(
            MutexGroupStates,
            "/mutex_group_states",
            self.mutex_group_states_cb,
            qos_profile=sub_qos,
        )

        self.create_subscription(
            String,
            "/fleet_state_update",
            self.fleet_state_update_cb,
            qos_profile=sub_qos,
        )

        self.create_timer(update_period, self.mutex_conflict_heartbeat)

    def mutex_group_manual_release_cb(self, msg: MutexGroupManualRelease):
        pass

    def mutex_group_states_cb(self, msg: MutexGroupStates):
        assignment: MutexGroupAssignment
        for assignment in msg.assignments:
            if assignment.claimant not in self.mutex_states:
                newMutex = MutexState()
                newMutex.claimant = assignment.claimant
                newMutex.claim_time = assignment.claim_time
                self.mutex_states.update({assignment.group: newMutex})
            else:
                mutexContext = self.mutex_states.get(assignment.group)
                mutexContext.claimant = assignment.claimant
                mutexContext.claim_time = assignment.claim_time

    def fleet_state_update_cb(self, msg: String):
        fleetStateUpdate = json.loads(msg.data)
        fleetData = fleetStateUpdate["data"]["robots"]
        for robot, data in fleetData.items():
            if robot not in self.robot_states:
                newRobot = RobotState()
                newRobot.name = data["name"]
                newRobot.task_id = data["task_id"]
                newRobot.unix_millis_time = data["unix_millis_time"]
                newRobot.mutex_locked = data["mutex_groups"]["locked"]
                newRobot.mutex_requesting = data["mutex_groups"]["requesting"]
                self.robot_states.update({robot: newRobot})
            else:
                robotContext = self.robot_states.get(robot)
                robotContext.task_id = data["task_id"]
                robotContext.unix_millis_time = data["unix_millis_time"]
                robotContext.mutex_locked = data["mutex_groups"]["locked"]
                robotContext.mutex_requesting = data["mutex_groups"]["requesting"]

    def mutex_conflict_heartbeat(self):
        for robotName, robotState in self.robot_states.items():
            for mutex in robotState.mutex_locked:
                isReceive, msg = rclpy.wait_for_message.wait_for_message(
                    MutexGroupRequest,
                    self,
                    "/mutex_group_request",
                    time_to_wait=5,
                )
                mutexContext = self.mutex_states.get(mutex, None)
                if mutexContext is not None:
                    if mutexContext.claimant == UNCLAIMED:
                        self.get_logger().error(
                            f"Robot [{robotName}] was locked mutex [{mutex}] but mutext state is UNCLAIMED!"
                        )
                        robotState.mutex_conflict = True


# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
def main(argv=None):
    # Init rclpy
    rclpy.init(args=argv)

    mutex_conflicts = MutexConflicts()
    rclpy.spin(mutex_conflicts)

    # executor = MultiThreadedExecutor()
    # executor.add_node(mutex_conflicts)
    # try:
    #     mutex_conflicts.get_logger().info("Beginning client, shut down with CTRL-C")
    #     executor.spin()
    # except KeyboardInterrupt:
    #     mutex_conflicts.get_logger().info("Keyboard interrupt, shutting down.\n")
    # mutex_conflicts.destroy_node()
    # rclpy.shutdown()
    # ////////////////////////////////////

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mutex_conflicts.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
