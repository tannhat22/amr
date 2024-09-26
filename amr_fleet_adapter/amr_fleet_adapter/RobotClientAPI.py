# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


"""
    The RobotAPI class is a wrapper for API calls to the robot. Here users
    are expected to fill up the implementations of functions which will be used
    by the RobotCommandHandle. For example, if your robot has a REST API, you
    will need to make http request calls to the appropriate endpoints within
    these functions.
"""
import requests
from urllib.error import HTTPError


class RobotAPI:
    # The constructor below accepts parameters typically required to submit
    # http requests. Users should modify the constructor as per the
    # requirements of their robot's API
    def __init__(self, prefix: str, user: str, password: str):
        self.prefix = prefix
        self.user = user
        self.password = password
        self.timeout = 5.0
        self.debug = False

    def check_connection(self):
        """Return True if connection to the robot API server is successful"""
        if self.data() is None:
            return False
        return True

    def robot_mode(self, robot_name: str):
        """Return robot state or None if any errors are encountered"""
        response = self.data(robot_name)
        if response is not None:
            if self.debug:
                print(f"Response: {response}")
            if not response["success"]:
                print(f"Response for {robot_name} was not successful")
                return None

            robot_mode = response["data"]["robot_mode"]
            return robot_mode

        print(f"No response received for {robot_name}")
        return None

    def position(self, robot_name: str):
        """Return [x, y, theta] expressed in the robot's coordinate frame or
        None if any errors are encountered"""
        response = self.data(robot_name)
        if response is not None:
            if self.debug:
                print(f"Response: {response}")
            if not response["success"]:
                print(f"Response for {robot_name} was not successful")
                return None

            position = response["data"]["position"]
            x = position["x"]
            y = position["y"]
            angle = position["yaw"]
            return [x, y, angle]

        print(f"No response received for {robot_name}")
        return None

    def navigate(
        self, robot_name: str, cmd_id: int, pose, map_name: str, speed_limit=0.0
    ):
        """Request the robot to navigate to pose:[x,y,theta] where x, y and
        and theta are in the robot's coordinate convention. This function
        should return True if the robot has accepted the request,
        else False"""
        assert len(pose) > 2
        url = (
            self.prefix + f"/vdm-rmf/cmd/navigate/?robot_name={robot_name}"
            f"&cmd_id={cmd_id}"
        )
        data = {}  # data fields: task, map_name, destination{}, data{}
        data["map_name"] = map_name
        data["destination"] = {"x": pose[0], "y": pose[1], "yaw": pose[2]}
        data["speed_limit"] = speed_limit
        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            response.raise_for_status()
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()["success"]
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return False

    def follow_path(self, robot_name: str, cmd_id: int, waypoints: list, map_name: str):
        """Request the robot to follow path to pose:[x,y,theta] where x, y and
        and theta are in the robot's coordinate convention. This function
        should return True if the robot has accepted the request,
        else False"""
        assert len(waypoints) >= 1
        url = (
            self.prefix + f"/vdm-rmf/cmd/follow_path/?robot_name={robot_name}"
            f"&cmd_id={cmd_id}"
        )
        data = {}  # data fields: task, map_name, destination{}, data{}
        data["map_name"] = map_name
        # data['speed_limit'] = speed_limit
        data["waypoints"] = [
            {
                "x": waypoint[0],
                "y": waypoint[1],
                "yaw": waypoint[2],
                "speed_limit": waypoint[3],
            }
            for waypoint in waypoints
        ]

        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            response.raise_for_status()
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()["success"]
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return False

    # def start_process(self,
    #                   robot_name: str,
    #                   cmd_id: int,
    #                   process: str,
    #                   map_name: str):
    #     ''' Request the robot to begin a process. This is specific to the robot
    #         and the use case. For example, load/unload a cart for Deliverybot
    #         or begin cleaning a zone for a cleaning robot.
    #         Return True if the robot has accepted the request, else False'''
    #     url = self.prefix +\
    #         f"/vdm-rmf/amr_fm/start_task?robot_name={robot_name}" \
    #         f"&cmd_id={cmd_id}"
    #     # data fields: task, map_name, destination{}, data{}
    #     data = {'task': process, 'map_name': map_name}
    #     try:
    #         response = requests.post(url, timeout=self.timeout, json=data)
    #         response.raise_for_status()
    #         if self.debug:
    #             print(f'Response: {response.json()}')
    #         return response.json()['success']
    #     except HTTPError as http_err:
    #         print(f'HTTP error: {http_err}')
    #     except Exception as err:
    #         print(f'Other error: {err}')
    #     return False

    def start_process(self, robot_name: str, cmd_id: int, map_name: str, process: dict):
        """Request the robot to begin a process. This is specific to the robot
        and the use case. For example, load/unload a cart for Deliverybot
        or begin cleaning a zone for a cleaning robot.
        Return True if the robot has accepted the request, else False"""
        url = (
            self.prefix + f"/vdm-rmf/cmd/start_task?robot_name={robot_name}"
            f"&cmd_id={cmd_id}"
        )
        # data fields: task, map_name, destination{}, data{}
        data = {"task": process, "map_name": map_name}
        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            response.raise_for_status()
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()["success"]
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return False

    def localize(self, robot_name: str, cmd_id: int, map_name: str, pose):
        """Request the robot to localize to pose:[x,y,theta] where x, y and
        and theta are in the robot's coordinate convention. This function
        should return True if the robot has accepted the request,
        else False"""
        assert len(pose) > 2
        url = (
            self.prefix + f"/vdm-rmf/cmd/localize/?robot_name={robot_name}"
            f"&cmd_id={cmd_id}"
        )
        data = {}  # data fields: task, map_name, destination{}, data{}
        data["map_name"] = map_name
        data["destination"] = {"x": pose[0], "y": pose[1], "yaw": pose[2]}
        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            response.raise_for_status()
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()["success"]
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return False

    # ///////////////////////////////////////////////////////////////////////////
    # ///////////////////////////////////////////////////////////////////////////
    def machine_data(self, machine_name=None):
        if machine_name is None:
            url = self.prefix + f"/vdm-rmf/machine_data/status/"
        else:
            url = (
                self.prefix
                + f"/vdm-rmf/machine_data/status?machine_name={machine_name}"
            )
        try:
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            if not self.debug:
                print(f"Response: {response.json()}")
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return None

    def machine_trigger(self, robot_name: str, cmd_id: int, process: dict):
        """Request the machine to begin a process
        Return True if the machine has accepted the request, else False"""
        url = (
            self.prefix + f"/vdm-rmf/cmd/machine_trigger?robot_name={robot_name}"
            f"&cmd_id={cmd_id}"
        )
        data = {"task": process}
        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            response.raise_for_status()
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()["success"]
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return False

    def station_trigger(self, robot_name: str, cmd_id: int, process: dict):
        """Request the station to begin a process
        Return True if the station has accepted the request, else False"""
        url = (
            self.prefix + f"/vdm-rmf/cmd/station_trigger?robot_name={robot_name}"
            f"&cmd_id={cmd_id}"
        )
        data = {"task": process}
        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            response.raise_for_status()
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()["success"]
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return False

    def machine_process_completed(self, machine_name: str, cmd_id: int):
        """Return True if the machine has successfully completed its previous
        process request. Else False."""
        response = self.machine_data(machine_name)
        if response is not None:
            data = response.get("data")
            if data is not None:
                completed = data["last_completed_request"] == str(cmd_id)
                if self.debug:
                    print(f"Response machine_process_completed: {completed}")
                return completed
        return False

    # ///////////////////////////////////////////////////////////////////////////
    # ///////////////////////////////////////////////////////////////////////////

    def charger_trigger(self, robot_name: str, cmd_id: int, process: dict):
        """Request the charger to begin a process
        Return True if the charger has accepted the request, else False"""
        url = (
            self.prefix + f"/vdm-rmf/cmd/charger_trigger?robot_name={robot_name}"
            f"&cmd_id={cmd_id}"
        )
        # data fields: task, map_name, destination{}, data{}
        data = {"task": process}
        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            response.raise_for_status()
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()["success"]
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return False

    def pause(self, robot_name: str, cmd_id: int):
        """Command the robot to pause.
        Return True if robot has successfully paused. Else False"""
        url = (
            self.prefix + f"/vdm-rmf/cmd/pause_robot?robot_name={robot_name}"
            f"&cmd_id={cmd_id}"
        )
        try:
            response = requests.get(url, self.timeout)
            response.raise_for_status()
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()["success"]
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return False

    def wait(self, robot_name: str, cmd_id: str):
        """Command the robot to wait.
        Return True if robot has successfully waited. Else False"""
        url = (
            self.prefix + f"/vdm-rmf/cmd/wait_robot?robot_name={robot_name}"
            f"&cmd_id={cmd_id}"
        )
        try:
            response = requests.get(url, self.timeout)
            response.raise_for_status()
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()["success"]
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return False

    def resume(self, robot_name: str, cmd_id: str):
        """Command the robot to resume.
        Return True if robot has successfully resumed. Else False"""
        url = (
            self.prefix + f"/vdm-rmf/cmd/resume_robot?robot_name={robot_name}"
            f"&cmd_id={cmd_id}"
        )
        try:
            response = requests.get(url, self.timeout)
            response.raise_for_status()
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()["success"]
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return False

    def stop(self, robot_name: str, cmd_id: int):
        """Command the robot to stop.
        Return True if robot has successfully stopped. Else False"""
        url = (
            self.prefix + f"/vdm-rmf/cmd/stop_robot?robot_name={robot_name}"
            f"&cmd_id={cmd_id}"
        )
        try:
            response = requests.get(url, self.timeout)
            response.raise_for_status()
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()["success"]
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return False

    # def is_task_queue_finished(self, robot_name: str):
    #     ''' Command the robot is finished task.
    #         Return True if robot has finished task. Else False'''
    #     url = self.prefix +\
    #         f'/vdm-rmf/cmd/is_task_queue_finished?robot_name={robot_name}'
    #     try:
    #         response = requests.get(url, self.timeout)
    #         response.raise_for_status()
    #         if self.debug:
    #             print(f'Response: {response.json()}')
    #         return response.json()['data']['task_finished']
    #     except HTTPError as http_err:
    #         print(f'HTTP error: {http_err}')
    #     except Exception as err:
    #         print(f'Other error: {err}')
    #     return False

    def remaining_path_length(self, robot_name: str):
        """Return remaining path of the robot"""
        response = self.data(robot_name)
        if response is not None:
            path = response["data"].get("path_length")
            return path

        return False

    def navigation_remaining_duration(self, robot_name: str, cmd_id: int):
        """Return the number of seconds remaining for the robot to reach its
        destination"""
        response = self.data(robot_name)
        if response is None:
            return None

        if response is not None:
            arrival = response["data"].get("destination_arrival")
            if arrival is not None:
                if arrival.get("cmd_id") != cmd_id:
                    return None
                return arrival.get("duration")
            else:
                return None

        else:
            return None

    def navigation_completed(self, robot_name: str, cmd_id: int):
        """Return True if the last request the robot successfully completed
        matches cmd_id. Else False."""
        response = self.data(robot_name)
        if response is not None:
            data = response.get("data")
            if data is not None:
                completed = data["last_completed_request"] == cmd_id
                if self.debug:
                    print(f"Response navigation_completed: {completed}")
                return completed

        return False

    # def task_completed(self, robot_name: str):
    #     ''' Return True if the last request the robot successfully completed
    #         matches cmd_id. Else False.'''
    #     return self.is_task_queue_finished(robot_name)

    def process_completed(self, robot_name: str, cmd_id: int):
        """Return True if the robot has successfully completed its previous
        process request. Else False."""
        resp = self.navigation_completed(robot_name, cmd_id)
        if self.debug:
            print(f"Response process_completed: {resp}")
        return resp

    def battery_soc(self, robot_name: str):
        """Return the state of charge of the robot as a value between 0.0
        and 1.0. Else return None if any errors are encountered"""
        response = self.data(robot_name)
        if response is not None:
            return response["data"]["battery"] / 100.0
        else:
            return None

    def requires_replan(self, robot_name: str):
        """Return whether the robot needs RMF to replan"""
        response = self.data(robot_name)
        if response is not None:
            return response["data"].get("replan", False)
        return False

    def toggle_action(self, robot_name: str, toggle: bool):
        """Request to toggle the robot's mode_teleop parameter.
        Return True if the toggle request is successful"""
        url = self.prefix + f"/vdm-rmf/cmd/toggle_action?robot_name={robot_name}"
        data = {"toggle": toggle}
        try:
            response = requests.post(url, timeout=self.timeout, json=data)
            response.raise_for_status()
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()["success"]
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return False

    def data(self, robot_name=None):
        if robot_name is None:
            url = self.prefix + f"/vdm-rmf/data/status/"
        else:
            url = self.prefix + f"/vdm-rmf/data/status?robot_name={robot_name}"
        try:
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            if self.debug:
                print(f"Response: {response.json()}")
            return response.json()
        except HTTPError as http_err:
            print(f"HTTP error: {http_err}")
        except Exception as err:
            print(f"Other error: {err}")
        return None
