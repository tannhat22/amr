import enum
from urllib.error import HTTPError

import requests


class MachineAPIResult(enum.IntEnum):
    SUCCESS = 0
    """The request was successful"""

    RETRY = 1
    """The client failed to connect but might succeed if you try again"""

    IMPOSSIBLE = 2
    """The client connected but something about the request is impossible"""


class MachineAPI:
    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their machine's API
    def __init__(self, prefix: str, user: str, password: str):
        self.prefix = prefix
        self.user = user
        self.password = password
        self.timeout = 5.0
        self.debug = False

    def check_connection(self):
        """Return True if connection to the machine API server is successful."""
        if self.get_data() is None:
            return False
        return True

    def machine_mode(self, machine_name: str):
        """Return machine state or None if any errors are encountered"""
        response = self.get_data(machine_name)
        if response is not None:
            if self.debug:
                print(f"Response: {response}")

            machine_mode = response.mode
            return machine_mode

        print(f"No response received for {machine_name}")
        return None

    def start_activity(
        self,
        machine_name: str,
        cmd_id: int,
        activity: str,
        mode: int,
    ):
        """
        Request the machine to begin a process.

        This is specific to the machine and the use case.
        For example, load/unload a cart for Deliverybot
        or begin cleaning a zone for a cleaning machine.
        """
        url = (
            self.prefix + f"/open-rmf/rmf_vdm_wm/start_activity?machine_name={machine_name}"
            f"&cmd_id={cmd_id}"
        )
        data = {
            "activity": activity,
            "mode": mode,
        }
        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            response.raise_for_status()
            if self.debug:
                print(f"Response: {response.json()}")

            if response.json()["success"]:
                return MachineAPIResult.SUCCESS

            # If we get a response with success=False, then
            return MachineAPIResult.IMPOSSIBLE
        except HTTPError as http_err:
            print(f"HTTP error for {machine_name} in start_activity: {http_err}")
        except Exception as err:
            print(f"Other error {machine_name} in start_activity: {err}")
        return MachineAPIResult.RETRY

    def get_data(self, machine_name: str | None = None):
        """
        Return a MachineUpdateData for one machine if a name is given.

        Otherwise return a list of MachineUpdateData for all machines.
        """
        if machine_name is None:
            url = self.prefix + "/open-rmf/rmf_vdm_wm/status"
        else:
            url = self.prefix + f"/open-rmf/rmf_vdm_wm/status?machine_name={machine_name}"
        try:
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            if self.debug:
                print(f"Response: {response.json()}")
            if machine_name is not None:
                return MachineUpdateData(response.json()["data"])
            else:
                all_machines = []
                for machine in response.json()["all_machines"]:
                    all_machines.append(MachineUpdateData(machine))
                return all_machines
        except HTTPError as http_err:
            print(f"HTTP error for {machine_name} in get_data: {http_err}")
        except Exception as err:
            # print(f"Other error for {machine_name} in get_data: {err}")
            pass
        return None


class MachineUpdateData:
    """Update data for a single machine."""

    def __init__(self, data):
        self.machine_name = data["machine_name"]
        self.mode = data["mode"]
        self.last_dispenser_completed_request = data["last_dispenser_completed_request"]
        self.last_ingestor_completed_request = data["last_ingestor_completed_request"]

    def is_command_dispenser_completed(self, cmd_id):
        if self.last_dispenser_completed_request is None:
            return True
        return self.last_dispenser_completed_request == cmd_id

    def is_command_ingestor_completed(self, cmd_id):
        if self.last_ingestor_completed_request is None:
            return True
        return self.last_ingestor_completed_request == cmd_id
