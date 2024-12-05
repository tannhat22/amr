import argparse
import time
import sys
import threading
import yaml
import re
import rclpy

from rclpy.node import Node

# from rclpy.executors import MultiThreadedExecutor
# from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
# from rcl_interfaces.msg import ParameterDescriptor
from machine_fleet_msgs.msg import (
    DeliveryItem,
    DeliveryParams,
    DeliveryRequest,
    FleetMachineState,
    FleetStationState,
    MachineState,
    StationState,
    StationRequest,
)


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


class MachineRequester:
    _lock: threading.Lock

    _destination_pickup: str
    _destination_dropoff: str

    def __init__(
        self,
        name: str,
        sku: str,
        dispenser: str,
        ingestor: str,
        pickup_stations: list[str],
        dropoff_stations: list[str],
    ) -> None:
        self.name = name
        self.dispenser = dispenser
        self.ingestor = ingestor
        self.pickup_stations = pickup_stations
        self.dropoff_stations = dropoff_stations
        self.delivery_item = DeliveryItem()
        self.delivery_item.sku = sku
        self.delivery_item.quantity = 1

        self._destination_pickup = ""
        self._destination_dropoff = ""

        self._lock = threading.Lock()

    def set_destination_pickup(self, destination_pickup: str) -> None:
        with self._lock:
            self._destination_pickup = destination_pickup

    def get_destination_pickup(self) -> str:
        return self._destination_pickup

    def set_destination_dropoff(self, destination_dropoff: str) -> None:
        with self._lock:
            self._destination_dropoff = destination_dropoff

    def get_destination_dropoff(self) -> str:
        return self._destination_dropoff


class StationRequester:
    _lock: threading.Lock

    _destination_dropoff: str

    def __init__(
        self,
        name: str,
        sku: str,
        dropoff_stations: list[str],
    ) -> None:
        self.name = name
        self.dropoff_stations = dropoff_stations
        self.delivery_item = DeliveryItem()
        self.delivery_item.sku = sku
        self.delivery_item.quantity = 1

        self._destination_dropoff = ""

        self._lock = threading.Lock()

    def set_destination_dropoff(self, destination_dropoff: str) -> None:
        with self._lock:
            self._destination_dropoff = destination_dropoff

    def get_destination_dropoff(self) -> str:
        return self._destination_dropoff


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

    def __init__(self, config, nav_graph) -> None:
        super().__init__("autotask_manager")

        self._pickup_context_dict = {}
        self._dropoff_context_dict = {}
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
        for machine_name, machine_config in task_requester_yaml["machines"].items():
            sku = machine_config["sku"]
            dispenser = machine_config["dispenser"]
            ingestor = machine_config["ingestor"]
            pks = machine_config["pickup_stations"]
            dos = machine_config["dropoff_stations"]
            self._mreq_context_dict.update(
                {
                    machine_name: MachineRequester(
                        name=machine_name,
                        sku=sku,
                        dispenser=dispenser,
                        ingestor=ingestor,
                        pickup_stations=pks,
                        dropoff_stations=dos,
                    )
                }
            )

        # Add station requester context
        self._sreq_context_dict = {}
        for station_name, station_config in task_requester_yaml["stations"].items():
            sku = station_config["sku"]
            dos = station_config["dropoff_stations"]
            self._sreq_context_dict.update(
                {
                    station_name: StationRequester(
                        name=station_name,
                        sku=sku,
                        dropoff_stations=dos,
                    )
                }
            )

        # Publishers:
        self._delivery_request_pub = self.create_publisher(
            DeliveryRequest, "amr_delivery_requests", 10
        )

        self._station_state_pub = self.create_publisher(FleetStationState, "station_states", 10)

        # Subcribers:
        self.create_subscription(
            FleetMachineState, "fleet_machine_state", self.fleet_machine_state_cb, 10
        )

        self.create_subscription(
            StationRequest, "station_requests", self.station_request_callback, 10
        )

        # Timers:
        self._pub_station_state_timer = self.create_timer(1.0, self._publish_station_states)

    def publish_delivery_requests(
        self, requester: str, start_time: int, pickup: list, dropoff: list
    ):
        deliveryParam = DeliveryParams()
        deliveryParam.pickup_place_name = pickup[0]
        deliveryParam.pickup_dispenser = pickup[1]
        deliveryParam.pickup_items = pickup[2]
        deliveryParam.dropoff_place_name = dropoff[0]
        deliveryParam.dropoff_ingestor = dropoff[1]
        deliveryParam.dropoff_items = dropoff[2]

        msg = DeliveryRequest()
        msg.requester = requester
        msg.start_time = start_time
        msg.delivery_params.append(deliveryParam)
        self._delivery_request_pub.publish(msg)

    def station_request_callback(self, request: StationRequest):
        stationContext = None
        if request.station_type == StationRequest.TYPE_PICKUP:
            stationContext = self._pickup_context_dict.get(request.station_name, None)
            if request.mode == StationState.MODE_EMPTY:
                # Thêm logic ghi trạng thái ngược xuống cho machine đang quản lý station này
                stationContext.reset()
        elif request.station_type == StationRequest.TYPE_DROPOFF:
            stationContext = self._dropoff_context_dict.get(request.station_name, None)
            if request.mode == StationRequest.MODE_FILLED:
                # Thêm logic ghi trạng thái ngược xuống cho machine đang quản lý station này
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
        for state in states:
            if state.machine_name in self._mreq_context_dict:
                requester = self._mreq_context_dict.get(state.machine_name)

                # Handle pickup request
                if state.request_pickup:
                    if requester.get_destination_dropoff() == "":
                        for dropoff_name in requester.dropoff_stations:
                            station_context = self._dropoff_context_dict.get(dropoff_name, None)
                            if (
                                station_context is not None
                                and station_context.get_state().mode == StationState.MODE_EMPTY
                                and station_context.set_occupant(requester.name)
                            ):
                                self.get_logger().info(
                                    f"detect pickup request from machine [{requester.name}] will send task for delivery it"
                                )
                                requester.set_destination_dropoff(dropoff_name)

                                pk_info = [
                                    requester.dispenser,
                                    requester.name,
                                    requester.delivery_item,
                                ]
                                do_info = [
                                    dropoff_name,
                                    station_context._handler,
                                    requester.delivery_item,
                                ]
                                self.publish_delivery_requests(
                                    requester=requester.name,
                                    start_time=0,
                                    pickup=pk_info,
                                    dropoff=do_info,
                                )
                                break
                else:
                    requester.set_destination_dropoff("")

                # Handle dropoff request
                if state.request_dropoff:
                    if requester.get_destination_pickup() == "":
                        for pickup_name in requester.pickup_stations:
                            station_context = self._pickup_context_dict.get(pickup_name, None)
                            if (
                                station_context is not None
                                and station_context.get_state().mode == StationState.MODE_FILLED
                                and station_context.set_occupant(requester.name)
                            ):
                                self.get_logger().info(
                                    f"detect dropoff request from machine [{requester.name}] will send task for delivery it"
                                )
                                requester.set_destination_pickup(pickup_name)

                                pk_info = [
                                    pickup_name,
                                    station_context._handler,
                                    requester.delivery_item,
                                ]
                                do_info = [
                                    requester.ingestor,
                                    requester.name,
                                    requester.delivery_item,
                                ]
                                self.publish_delivery_requests(
                                    requester=requester.name,
                                    start_time=0,
                                    pickup=pk_info,
                                    dropoff=do_info,
                                )
                                break
                else:
                    requester.set_destination_pickup("")

            # Handles station state
            station: StationState
            for station in state.station_states:
                pk_context = self._pickup_context_dict.get(station.station_name, None)
                if pk_context is not None and not pk_context._is_occupied:
                    pk_context.set_state(station.mode)

                do_context = self._dropoff_context_dict.get(station.station_name, None)
                if do_context is not None and not do_context._is_occupied:
                    do_context.set_state(station.mode)

    def _publish_station_states(self):
        current_time = self.get_clock().now().to_msg()
        pickup_stations = []
        dropoff_stations = []
        for pk_name, pk_context in self._pickup_context_dict.items():
            pk_state = pk_context.get_state()
            pickup_stations.append(pk_state)

            # Gửi nhiệm vụ tự động khi phát hiện có hàng ở các trạm pickup requester:
            if pk_name in self._sreq_context_dict:
                requester = self._sreq_context_dict.get(pk_name)
                if pk_state.mode == StationState.MODE_FILLED:
                    if requester.get_destination_dropoff() == "":
                        for station_name in requester.dropoff_stations:
                            station_context = self._dropoff_context_dict.get(station_name, None)
                            if (
                                station_context is not None
                                and station_context.get_state().mode == StationState.MODE_EMPTY
                                and station_context.set_occupant(requester.name)
                            ):
                                self.get_logger().info(
                                    f"detect cart in station [{requester.name}] will send task for delivery it"
                                )
                                requester.set_destination_dropoff(station_name)

                                pk_info = [
                                    requester.name,
                                    pk_context._handler,
                                    requester.delivery_item,
                                ]
                                do_info = [
                                    station_name,
                                    station_context._handler,
                                    requester.delivery_item,
                                ]
                                self.publish_delivery_requests(
                                    requester=requester.name,
                                    start_time=0,
                                    pickup=pk_info,
                                    dropoff=do_info,
                                )
                                break
                else:
                    requester.set_destination_dropoff("")

        for do_context in self._dropoff_context_dict.values():
            do_state = do_context.get_state()
            dropoff_stations.append(do_state)

        msg = FleetStationState()
        msg.time = current_time
        msg.pickup_stations = pickup_stations
        msg.dropoff_stations = dropoff_stations
        self._station_state_pub.publish(msg)


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
        "-n",
        "--nav_graph",
        type=str,
        required=True,
        help="Path to the nav_graph for this autotask manager",
    )

    args = parser.parse_args(args_without_ros[1:])

    config_path = args.config_file
    nav_graph_path = args.nav_graph

    # Parse the yaml in Python to get the autotask_manager info
    with open(config_path, "r") as f:
        config_yaml = yaml.safe_load(f)

    with open(nav_graph_path, "r") as f:
        nav_graph = yaml.safe_load(f)

    time.sleep(1.0)

    autotask_manager = AutoTaskManager(config_yaml, nav_graph)
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
    main()
