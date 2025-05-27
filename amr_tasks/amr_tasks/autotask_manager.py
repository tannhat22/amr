import sys
import time
import argparse
import threading
import yaml
import re
import rclpy
import json
import uuid

# from asyncio import Future
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSReliabilityPolicy as Reliability


# from rclpy.executors import MultiThreadedExecutor
# from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import String
from rmf_task_msgs.msg import ApiRequest, ApiResponse
from rmf_fleet_msgs.msg import MutexGroupStates, MutexGroupAssignment
from machine_fleet_msgs.msg import (
    DeliveryItem,
    DeliveryParams,
    DeliveryRequest,
    DeviceMode,
    FleetMachineState,
    FleetStationState,
    MachineState,
    MachineRequest,
    StationState,
    StationRequest,
)

TASK_FAILED = ["error", "failed", "canceled", "killed"]
TASK_NO_ACTIVE = ["error", "failed", "canceled", "killed", "completed"]
MUTEX_NO_CLAIMANT = 18446744073709551615


class StationContext:
    _is_occupied: bool
    _occupant_id: str
    _handler: str
    _lock: threading.Lock

    _state: StationState

    def __init__(self, name: str, handler: str) -> None:
        self._is_occupied = False
        self._occupant_id = ""
        self._lock = threading.Lock()

        self._handler = handler
        self._state = StationState()
        self._state.station_name = name

    def reset(self) -> None:
        with self._lock:
            self._is_occupied = False
            self._occupant_id = ""

    def set_occupant(self, occupant_id: str) -> bool:
        with self._lock:
            if self._is_occupied and self._occupant_id != occupant_id:
                return False

        self._is_occupied = True
        self._occupant_id = occupant_id
        return True

    def get_occupant(self) -> str:
        return self._occupant_id

    def set_state(self, mode: int) -> None:
        with self._lock:
            self._state.mode = mode

    def get_state(self) -> StationState:
        return self._state


class DeliveryStep:
    def __init__(
        self,
        sku: str,
        pickup_station: StationContext,
        dropoff_stations: list[StationContext],
    ):
        self.item = DeliveryItem()
        self.item.sku = sku
        self.item.quantity = 1
        self.pickup_station = pickup_station
        self.dropoff_stations = dropoff_stations


class TaskState:
    delivery_params: list[DeliveryParams]

    def __init__(self, id, submit_time, last_update, state, delivery_params):
        self.id = id
        self.submit_time = submit_time
        self.last_update = last_update
        self.state = state
        self.delivery_params = delivery_params


class MachineRequester:
    _lock: threading.Lock
    _current_task: TaskState | None

    def __init__(
        self,
        name: str,
        mode_operation: str,
        sku: str = None,
        dispenser: str = None,
        ingestor: str = None,
        pickup_stations: list[StationContext] = [],
        dropoff_stations: list[StationContext] = [],
        station_names: list[str] = [],
    ) -> None:
        self.name = name
        self.mode_operation = mode_operation
        self.dispenser = dispenser
        self.ingestor = ingestor
        self.pickup_stations = pickup_stations
        self.dropoff_stations = dropoff_stations
        self.delivery_item = DeliveryItem()
        self.delivery_item.sku = sku
        self.delivery_item.quantity = 1
        self.station_names = station_names

        self._current_task = None
        self._lock = threading.Lock()

    def set_current_task(self, current_task: TaskState | None) -> None:
        with self._lock:
            self._current_task = current_task

    def get_current_task(self) -> TaskState | None:
        return self._current_task


class StationRequester:
    _lock: threading.Lock
    _current_task: TaskState | None

    def __init__(self, name: str, delivery_steps: list[DeliveryStep]) -> None:
        self.name = name
        self.delivery_steps = delivery_steps
        self._current_task = None
        self._lock = threading.Lock()

    def set_current_task(self, current_task: TaskState | None) -> None:
        with self._lock:
            self._current_task = current_task

    def get_current_task(self) -> TaskState | None:
        return self._current_task


def search_mode_docking(dock_name: str):
    match = re.search(r"--(.+)", dock_name)
    if match:
        result = match.group(1)
        return result
    else:
        return None


class AutoTaskManager(Node):
    _pickup_context_dict: dict[str, StationContext]
    _dropoff_context_dict: dict[str, StationContext]
    _mreq_context_dict: dict[str, MachineRequester]
    _sreq_context_dict: dict[str, StationRequester]

    def __init__(self, config, nav_graphs):
        super().__init__("autotask_manager")

        # cb_group = MutuallyExclusiveCallbackGroup()

        self._pickup_context_dict = {}
        self._dropoff_context_dict = {}
        self.mutex_RF370CB_is_locked = False

        for nav_graph in nav_graphs:
            if nav_graph is None:
                continue

            for level in nav_graph["levels"]:
                for wp in nav_graph["levels"][level]["vertices"]:
                    assert len(wp) == 3, "Vertical structure not match, please check!"

                    if "dock_name" in wp[2]:
                        dock_name = wp[2]["dock_name"]
                        mode_dock = search_mode_docking(dock_name)
                        if mode_dock == "mpickup" or mode_dock == "mdropoff":
                            continue

                        if "pickup_dispenser" in wp[2]:
                            self._pickup_context_dict.update(
                                {dock_name: StationContext(dock_name, wp[2]["pickup_dispenser"])}
                            )
                        elif "dropoff_ingestor" in wp[2]:
                            self._dropoff_context_dict.update(
                                {dock_name: StationContext(dock_name, wp[2]["dropoff_ingestor"])}
                            )

        task_requester_yaml = config["TaskRequester"]

        # Add machine requester context
        self._mreq_context_dict = {}
        if task_requester_yaml["machines"] is not None:
            for machine_name, machine_config in task_requester_yaml["machines"].items():
                mode_operation = machine_config["mode_operation"]
                station_names = machine_config["station_names"]

                if mode_operation == "combine":
                    sku = machine_config["sku"]
                    dispenser = machine_config["dispenser"]
                    ingestor = machine_config["ingestor"]
                    pkss = machine_config["pickup_stations"]
                    doss = machine_config["dropoff_stations"]

                    pkss_context = []
                    for pks in pkss:
                        pks_context = self._pickup_context_dict.get(pks, None)
                        assert (
                            pks_context is not None
                        ), f"pickup_station [{pks}] not match with nav_graph"
                        pkss_context.append(pks_context)

                    doss_context = []
                    for dos in doss:
                        dos_context = self._dropoff_context_dict.get(dos, None)
                        assert (
                            dos_context is not None
                        ), f"dropoff_station [{dos}] not match with nav_graph"
                        doss_context.append(dos_context)

                    self._mreq_context_dict.update(
                        {
                            machine_name: MachineRequester(
                                name=machine_name,
                                mode_operation=mode_operation,
                                sku=sku,
                                dispenser=dispenser,
                                ingestor=ingestor,
                                pickup_stations=pkss_context,
                                dropoff_stations=doss_context,
                                station_names=station_names,
                            )
                        }
                    )
                else:
                    self._mreq_context_dict.update(
                        {
                            machine_name: MachineRequester(
                                name=machine_name,
                                mode_operation=mode_operation,
                                station_names=station_names,
                            )
                        }
                    )

        # Add station requester context
        self._sreq_context_dict = {}
        if task_requester_yaml["stations"] is not None:
            for station_name, station_config in task_requester_yaml["stations"].items():
                deliverySteps = []
                for step, step_config in station_config.items():
                    sku = step_config["sku"]
                    pks = step_config["pickup_station"]
                    doss = step_config["dropoff_stations"]
                    pks_context = self._pickup_context_dict.get(pks, None)
                    assert (
                        pks_context is not None
                    ), f"pickup_station [{pks}] not match with nav_graph"

                    doss_context = []
                    for dos in doss:
                        dos_context = self._dropoff_context_dict.get(dos, None)
                        assert (
                            dos_context is not None
                        ), f"dropoff_station [{dos}] not match with nav_graph"
                        doss_context.append(dos_context)

                    deliverySteps.append(DeliveryStep(sku, pks_context, doss_context))
                self._sreq_context_dict.update(
                    {
                        station_name: StationRequester(
                            name=station_name, delivery_steps=deliverySteps
                        )
                    }
                )

        transient_qos = QoSProfile(
            history=History.KEEP_LAST,
            depth=1,
            reliability=Reliability.RELIABLE,
            durability=Durability.TRANSIENT_LOCAL,
        )

        # Publishers:
        self.task_api_req_pub = self.create_publisher(
            ApiRequest, "task_api_requests", transient_qos
        )

        self._adapter_station_request_pub = self.create_publisher(
            StationRequest, "/adapter_station_requests", qos_profile=qos_profile_system_default
        )

        self.station_state_pub = self.create_publisher(
            FleetStationState, "/station_states", qos_profile=qos_profile_system_default
        )
        self.machine_req_pub = self.create_publisher(
            MachineRequest,
            "adapter_machine_requests",
            qos_profile=qos_profile_system_default,
        )

        # Subcribers:
        self.create_subscription(
            StationRequest,
            "/station_requests",
            self.station_request_callback,
            qos_profile=qos_profile_system_default,
        )

        self.create_subscription(
            FleetMachineState,
            "/fleet_machine_state",
            self.fleet_machine_state_cb,
            qos_profile=qos_profile_system_default,
        )

        self.create_subscription(
            String,
            "/task_state_update",
            self.task_state_update_cb,
            qos_profile=qos_profile_system_default,
        )

        self.create_subscription(
            MutexGroupStates,
            "/mutex_group_states",
            self.mutex_group_states_cb,
            qos_profile=qos_profile_system_default,
        )

        # Timers:
        self.create_timer(1.0, self.publish_station_states)

        self.get_logger().info("Beginning client, shut down with CTRL-C")

    # pickup_descriptions
    def __create_pickup_desc(self, pickup: DeliveryParams):
        place = pickup.pickup_place_name
        handler = pickup.pickup_dispenser
        payload = [{"sku": pickup.pickup_items.sku, "quantity": pickup.pickup_items.quantity}]

        return {
            "place": place,
            "handler": handler,
            "payload": payload,
        }

    # dropoff_descriptions
    def __create_dropoff_desc(self, dropoff: DeliveryParams):
        place = dropoff.dropoff_place_name
        handler = dropoff.dropoff_ingestor
        payload = [{"sku": dropoff.dropoff_items.sku, "quantity": dropoff.dropoff_items.quantity}]

        return {
            "place": place,
            "handler": handler,
            "payload": payload,
        }

    def dispatch_delivery(
        self,
        fleet: str = None,
        robot: str = None,
        start_time_task: int = 0,
        requester: str = "amr_task",
        delivery_params: list[DeliveryParams] = [],
    ):
        assert len(delivery_params) > 0, "delivery_params invalid, please check!"

        # Construct task
        msg = ApiRequest()
        msg.request_id = "delivery_" + str(uuid.uuid4())
        payload = {}
        if fleet and robot:
            self.get_logger().info("Using 'robot_task_request'")
            payload["type"] = "robot_task_request"
            payload["fleet"] = fleet
            payload["robot"] = robot
        else:
            self.get_logger().info("Using 'dispatch_task_request'")
            payload["type"] = "dispatch_task_request"
        request = {}

        # Set task request request time, start time and requester
        now = self.get_clock().now().to_msg()
        now.sec = now.sec + start_time_task
        start_time = now.sec * 1000 + round(now.nanosec / 10**6)
        request["unix_millis_request_time"] = start_time
        request["unix_millis_earliest_start_time"] = start_time
        request["priority"] = {"type": "binary", "value": 1}
        request["requester"] = requester

        if fleet:
            request["fleet_name"] = fleet

        # Use standard delivery task type
        if len(delivery_params) == 1:
            request["category"] = "delivery"
            request["labels"] = [
                "task_definition_id=delivery",
                f"pickup={delivery_params[0].pickup_place_name}",
                f"destination={delivery_params[0].dropoff_place_name}",
                f"cart_id={delivery_params[0].dropoff_items.sku}",
            ]
            description = {
                "pickup": self.__create_pickup_desc(delivery_params[0]),
                "dropoff": self.__create_dropoff_desc(delivery_params[0]),
            }
        else:
            # Define multi_delivery with request category compose
            request["category"] = "compose"
            request["labels"] = [
                "task_definition_id=delivery_multiple",
                f"pickup={delivery_params[0].pickup_place_name}",
                f"destination={delivery_params[0].dropoff_place_name}",
                f"cart_id={delivery_params[0].dropoff_items.sku}",
            ]

            # Define task request description with phases
            description = {}
            description["category"] = "multi_delivery"
            # description["detail"] = "Delivery rotor TTR and STR"
            description["phases"] = []
            for i in range(0, len(delivery_params)):
                # Add each pickup activity to phases
                description["phases"].append(
                    {
                        "activity": {
                            "category": "pickup",
                            "description": self.__create_pickup_desc(delivery_params[i]),
                        }
                    }
                )
                # Add each dropoff activity to phases
                description["phases"].append(
                    {
                        "activity": {
                            "category": "dropoff",
                            "description": self.__create_dropoff_desc(delivery_params[i]),
                        }
                    }
                )

        request["description"] = description
        payload["request"] = request
        msg.json_msg = json.dumps(payload)

        # print(f"Json msg payload: \n{json.dumps(payload, indent=2)}")
        self.task_api_req_pub.publish(msg)
        return msg.request_id

    def mutex_group_states_cb(self, msg: MutexGroupStates):
        assignment: MutexGroupAssignment
        for assignment in msg.assignments:
            if assignment.group == "zone_RF370CB":
                if assignment.claimant == MUTEX_NO_CLAIMANT:
                    self.mutex_RF370CB_is_locked = False
                else:
                    self.mutex_RF370CB_is_locked = True
                break

    def task_state_update_cb(self, msg: String):
        current_time = self.get_clock().now()
        taskState = json.loads(msg.data)
        requester = taskState["data"]["booking"]["requester"]
        taskId = taskState["data"]["booking"]["id"]
        status = taskState["data"]["status"]

        if requester in self._sreq_context_dict:
            requesterContext = self._sreq_context_dict.get(requester)
            currentTask = requesterContext.get_current_task()
            if currentTask is not None:
                currentTask.state = status
                currentTask.last_update = current_time
                if status in TASK_FAILED:
                    for param in currentTask.delivery_params:
                        do_station = self._dropoff_context_dict.get(param.dropoff_place_name)
                        if do_station.get_occupant() == requester:
                            self.get_logger().warn(
                                f"Detect autotask from [{requester}] was {status} (task_id: {taskId}), reset common dropoff station [{param.dropoff_place_name}]!"
                            )
                            do_station.reset()
            # else:
            #     self.get_logger().warn(
            #         f"Requester [{requester}] current task is None, please check for debugging!"
            #     )

        elif requester in self._mreq_context_dict:
            requesterContext = self._mreq_context_dict.get(requester)
            currentTask = requesterContext.get_current_task()
            if currentTask is not None:
                currentTask.state = status
                currentTask.last_update = current_time
                if status in TASK_FAILED:
                    machineReq = MachineRequest()
                    machineReq.machine_name = requester
                    machineReq.time = current_time.to_msg()
                    if status == "canceled":
                        machineReq.request_mode.mode = DeviceMode.MODE_CANCEL
                    else:
                        machineReq.request_mode.mode = DeviceMode.MODE_ROBOT_ERROR

                    self.get_logger().warn(
                        f"Detect autotask from [{requester}] was {status} (task_id: {taskId})!"
                    )

                    for param in currentTask.delivery_params:
                        pk_station = self._pickup_context_dict.get(param.pickup_place_name, None)
                        do_station = self._dropoff_context_dict.get(param.dropoff_place_name, None)
                        if pk_station is not None and pk_station.get_occupant() == requester:
                            self.get_logger().warn(
                                f"Reset common pickup station [{param.pickup_place_name}]!"
                            )
                            pk_station.reset()
                        if do_station is not None and do_station.get_occupant() == requester:
                            self.get_logger().warn(
                                f"Reset common dropoff station [{param.dropoff_place_name}]!"
                            )
                            do_station.reset()

                    if (
                        currentTask.delivery_params[0].pickup_place_name
                        == requesterContext.dispenser
                    ):
                        machineReq.request_type = MachineRequest.REQUEST_DISPENSER
                    else:
                        machineReq.request_type = MachineRequest.REQUEST_INGESTOR

                    self.get_logger().warn(
                        f"Response [{status}] to [{requesterContext.name}] (request_type: {machineReq.request_type})!"
                    )
                    self.machine_req_pub.publish(machineReq)
            # else:
            #     self.get_logger().warn(
            #         f"Requester [{requester}] current task is None, please check for debugging!"
            #     )
        else:
            return

    def station_request_callback(self, request: StationRequest):
        stationContext = None
        for machine_config in self._mreq_context_dict.values():
            if request.station_name in machine_config.station_names:
                msg = StationRequest()
                msg.time = self.get_clock().now().to_msg()
                msg.machine_name = machine_config.name
                msg.station_name = request.station_name
                msg.station_type = request.station_type
                msg.mode = request.mode
                self._adapter_station_request_pub.publish(msg)
                return

        if request.station_type == StationRequest.TYPE_PICKUP:
            stationContext = self._pickup_context_dict.get(request.station_name, None)
            if request.mode == StationState.MODE_EMPTY:
                stationContext.reset()
        elif request.station_type == StationRequest.TYPE_DROPOFF:
            stationContext = self._dropoff_context_dict.get(request.station_name, None)
            if request.mode == StationRequest.MODE_FILLED:
                stationContext.reset()

        if stationContext is None:
            self.get_logger().error(
                f"received station request for [{request.station_name}] but not station not found!"
            )
            return

        stationContext.set_state(mode=request.mode)
        return

    def fleet_machine_state_cb(self, states: FleetMachineState):
        state: MachineState
        for state in states.machines:
            if state.machine_name in self._mreq_context_dict:
                requester = self._mreq_context_dict.get(state.machine_name)
                if requester.mode_operation == "combine":
                    current_time = self.get_clock().now()
                    currentTask = requester.get_current_task()
                    # Handle pickup request
                    if state.request_pickup:
                        if currentTask is None:
                            for station_context in requester.dropoff_stations:
                                if (
                                    station_context.get_state().mode == StationState.MODE_EMPTY
                                    and station_context.set_occupant(requester.name)
                                ):
                                    param = DeliveryParams()
                                    param.pickup_items = requester.delivery_item
                                    param.pickup_dispenser = requester.name
                                    param.pickup_place_name = requester.dispenser
                                    param.dropoff_items = requester.delivery_item
                                    param.dropoff_ingestor = station_context._handler
                                    param.dropoff_place_name = (
                                        station_context.get_state().station_name
                                    )
                                    self.get_logger().warn(
                                        f"detect pickup request from machine [{requester.name}], send task delivery "
                                        f"(pickup: {param.pickup_place_name} -> dropoff: {param.dropoff_place_name})!"
                                    )

                                    taskId = self.dispatch_delivery(
                                        start_time_task=0,
                                        requester=requester.name,
                                        delivery_params=[param],
                                    )
                                    requester.set_current_task(
                                        TaskState(
                                            id=taskId,
                                            submit_time=current_time,
                                            last_update=current_time,
                                            state="queued",
                                            delivery_params=[param],
                                        )
                                    )
                                    break

                    elif (
                        currentTask is not None
                        and currentTask.delivery_params[0].pickup_place_name == requester.dispenser
                        and currentTask.state in TASK_NO_ACTIVE
                    ):
                        requester.set_current_task(None)

                    # Handle dropoff request
                    if state.request_dropoff:
                        if currentTask is None:
                            for station_context in requester.pickup_stations:
                                if (
                                    station_context.get_state().mode == StationState.MODE_FILLED
                                    and station_context.set_occupant(requester.name)
                                ):
                                    param = DeliveryParams()
                                    param.pickup_items = requester.delivery_item
                                    param.pickup_dispenser = station_context._handler
                                    param.pickup_place_name = (
                                        station_context.get_state().station_name
                                    )
                                    param.dropoff_items = requester.delivery_item
                                    param.dropoff_ingestor = requester.name
                                    param.dropoff_place_name = requester.ingestor
                                    self.get_logger().warn(
                                        f"detect dropoff request from machine [{requester.name}], send task delivery "
                                        f"(pickup: {param.pickup_place_name} -> dropoff: {param.dropoff_place_name})!"
                                    )

                                    taskId = self.dispatch_delivery(
                                        start_time_task=0,
                                        requester=requester.name,
                                        delivery_params=[param],
                                    )
                                    requester.set_current_task(
                                        TaskState(
                                            id=taskId,
                                            submit_time=current_time,
                                            last_update=current_time,
                                            state="queued",
                                            delivery_params=[param],
                                        )
                                    )
                                    break

                    elif (
                        currentTask is not None
                        and currentTask.delivery_params[0].dropoff_place_name == requester.ingestor
                        and currentTask.state in TASK_NO_ACTIVE
                    ):
                        requester.set_current_task(None)

                # Handles station state
                station: StationState
                for station in state.station_states:
                    pk_context = self._pickup_context_dict.get(station.station_name, None)
                    do_context = self._dropoff_context_dict.get(station.station_name, None)
                    if pk_context is not None:
                        if station.mode == StationState.MODE_EMPTY:
                            pk_context.reset()
                        pk_context.set_state(station.mode)
                    elif do_context is not None:
                        if station.mode == StationState.MODE_FILLED:
                            do_context.reset()
                        do_context.set_state(station.mode)
                    else:
                        continue

    def publish_station_states(self):
        current_time = self.get_clock().now()
        pickup_stations = []
        dropoff_stations = []
        for pk_name, pk_context in self._pickup_context_dict.items():
            pk_state = pk_context.get_state()
            pickup_stations.append(pk_state)

            # Gửi nhiệm vụ tự động khi phát hiện có hàng ở các trạm pickup requester:
            if pk_name in self._sreq_context_dict:
                requester = self._sreq_context_dict.get(pk_name)
                currentTask = requester.get_current_task()
                if pk_state.mode == StationState.MODE_FILLED:
                    if currentTask is None:
                        if self.mutex_RF370CB_is_locked:
                            self.get_logger().warn(
                                f"detect cart in requester station: [{requester.name}] but mutex group zone_RF370CB is locked, will wait for mutex is released!"
                            )
                            continue

                        deliveryParams = []
                        for step in requester.delivery_steps:
                            param = DeliveryParams()
                            param.pickup_items = step.item
                            param.pickup_dispenser = step.pickup_station._handler
                            param.pickup_place_name = step.pickup_station.get_state().station_name
                            param.dropoff_items = step.item
                            if len(step.dropoff_stations) == 1:
                                param.dropoff_ingestor = step.dropoff_stations[0]._handler
                                param.dropoff_place_name = (
                                    step.dropoff_stations[0].get_state().station_name
                                )
                                deliveryParams.append(param)
                            else:
                                found_station_empty = False
                                for station_context in step.dropoff_stations:
                                    if (
                                        station_context.get_state().mode == StationState.MODE_EMPTY
                                        and station_context.set_occupant(requester.name)
                                    ):
                                        param.dropoff_ingestor = station_context._handler
                                        param.dropoff_place_name = (
                                            station_context.get_state().station_name
                                        )
                                        deliveryParams.append(param)
                                        found_station_empty = True
                                        break
                                if not found_station_empty:
                                    break

                        if len(requester.delivery_steps) == len(deliveryParams):
                            information = ""
                            for param in deliveryParams:
                                information += f"pickup: {param.pickup_place_name} -> dropoff: {param.dropoff_place_name} ->"
                            information = information[:-3]
                            self.get_logger().warn(
                                f"detect cart in requester station: [{requester.name}], send task delivery ({information})!"
                            )

                            taskId = self.dispatch_delivery(
                                start_time_task=0,
                                requester=requester.name,
                                delivery_params=deliveryParams,
                            )
                            requester.set_current_task(
                                TaskState(
                                    id=taskId,
                                    submit_time=current_time,
                                    last_update=current_time,
                                    state="queued",
                                    delivery_params=deliveryParams,
                                )
                            )
                    elif currentTask.state == "completed":
                        self.get_logger().warn(
                            f"detect cart in requester station: [{requester.name}] and current task was completed, reset current task!"
                        )
                        requester.set_current_task(None)

                elif currentTask is not None and currentTask.state in TASK_NO_ACTIVE:
                    requester.set_current_task(None)

        for do_context in self._dropoff_context_dict.values():
            do_state = do_context.get_state()
            dropoff_stations.append(do_state)

        msg = FleetStationState()
        msg.time = current_time.to_msg()
        msg.pickup_stations = pickup_stations
        msg.dropoff_stations = dropoff_stations
        self.station_state_pub.publish(msg)


def main(argv=sys.argv):
    rclpy.init(args=argv)
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="autotask_manager",
        description="Configure and spin up the autotask manager",
    )
    parser.add_argument(
        "-c",
        "--config_file",
        type=str,
        required=True,
        help="Path to the config.yaml file",
    )
    parser.add_argument(
        "-n1",
        "--nav_graph_1_file",
        type=str,
        required=True,
        help="Path to the nav_graph_1_file for this autotask manager",
    )
    parser.add_argument(
        "-n2",
        "--nav_graph_2_file",
        type=str,
        required=True,
        help="Path to the nav_graph_2_file for this autotask manager",
    )

    args = parser.parse_args(args_without_ros[1:])

    config_path = args.config_file
    nav_graph_1_path = args.nav_graph_1_file
    nav_graph_2_path = args.nav_graph_2_file

    # Parse the yaml in Python to get the autotask_manager info
    with open(config_path, "r") as f:
        config_yaml = yaml.safe_load(f)

    if nav_graph_1_path == "":
        nav_graph_1 = None
    else:
        with open(nav_graph_1_path, "r") as f:
            nav_graph_1 = yaml.safe_load(f)

    if nav_graph_2_path == "":
        nav_graph_2 = None
    else:
        with open(nav_graph_2_path, "r") as f:
            nav_graph_2 = yaml.safe_load(f)

    time.sleep(1.0)

    autotask_manager = AutoTaskManager(config_yaml, [nav_graph_1, nav_graph_2])
    rclpy.spin(autotask_manager)

    autotask_manager.destroy_node()
    rclpy.shutdown()

    # executor = MultiThreadedExecutor()
    # executor.add_node(node=autotask_manager)

    # try:
    #     autotask_manager.get_logger().info("Beginning client, shut down with CTRL-C")
    #     executor.spin()
    # except KeyboardInterrupt:
    #     autotask_manager.get_logger().info("Keyboard interrupt, shutting down.\n")
    # autotask_manager.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
