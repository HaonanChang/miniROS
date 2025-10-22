"""
Test the frequency of pika reading.
"""

import glob
import json
import math
import re
import struct
import threading
import time

import serial
from enum import Enum
from loguru import logger
import numpy as np



class PIKACommandType(Enum):
    DISABLE = 10
    ENABLE = 11
    SET_ZERO = 12
    CURRENT = 15
    EFFORT_CTRL = 20
    VELOCITY_CTRL = 21
    POSITION_CTRL = 22


class PikaGripperClient:
    def __init__(
        self,
        port: str | None = None,
        baudrate: int = 460800,
        max_motor_torque_current: float = 2.0,
        timeout: float = 1.0,
    ) -> None:
        super().__init__()
        if port is None:
            # Find the first available port
            ports = glob.glob("/dev/ttyUSB*")
            if not ports:
                raise RuntimeError("No serial ports found.")
            if len(ports) > 1:
                logger.info("Multiple serial ports found. Using the first one.")
            port = ports[0]

        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        self._buffer: str = ""
        self._motor_data: dict[str, float | str] = {
            "Speed": 0.0,  # current speed（rad/s）
            "Current": 0,  # motor current（mA）
            "Position": 0.0,  # motor position（rad）
        }
        self._motor_status: dict[str, float | str] = {
            "Voltage": 0.0,  # motor driver voltage(V)
            "DriverTemp": 0,  # motor driver temperature(°C)
            "MotorTemp": 0,  # motor temperature(°C)
            "Status": "0x00",  # motor driver status
            "BusCurrent": 0,  # bus current(mA)
        }
        self._shutdown_event = threading.Event()
        self._distance_0 = self.get_distance(0)
        self.set_motor_torque(max_motor_torque_current)

    def _send_command(self, command_type: PIKACommandType, value: float = 0.0) -> bool:
        try:
            data = bytearray()
            data.append(int(command_type.value))  # command type

            # add the value to the data
            value_bytes = bytearray(struct.pack("<f", value))
            data.extend(value_bytes)

            # add the end of the command
            data.extend(b"\r\n")

            self.serial.write(data)
            self.serial.flush()
            return True
        except Exception as e:
            logger.error("Error sending command: {}", e)
            return False

    def _find_json_data(self) -> dict[str, dict[str, float | str]] | None:
        cnt = 0
        end = 0
        for i in range(len(self._buffer) - 1, -1, -1):
            if self._buffer[i] == "}":
                cnt += 1
                if cnt == 1:
                    end = i
            elif self._buffer[i] == "{":
                if cnt == 0:
                    continue
                else:
                    cnt -= 1
                    if cnt == 0:
                        try:
                            start = i
                            json_str = self._buffer[start : end + 1]
                            # remove the un needed comma
                            cleaned_json_str = re.sub(r",\s*}", "}", json_str)
                            cleaned_json_str = re.sub(r",\s*\]", "]", cleaned_json_str)

                            json_data = json.loads(cleaned_json_str)
                            assert isinstance(json_data, dict)
                            if "motor" in json_data and "motorstatus" in json_data:
                                if (
                                    "Current" in json_data["motor"]
                                    and "Position" in json_data["motor"]
                                ):
                                    self._buffer = self._buffer[start:]
                                    return json_data
                        except Exception:
                            pass
        return None

    def get_distance(self, angle: float) -> float:
        angle = (180.0 - 43.99) / 180.0 * math.pi - angle
        height = 0.0325 * math.sin(angle)
        width_d = 0.0325 * math.cos(angle)
        width = math.sqrt(0.058**2 - (height - 0.01456) ** 2) + width_d
        return width

    def start(self) -> None:
        self._thread = threading.Thread(target=self._get_data)
        self._thread.start()
        time.sleep(1)
        self._send_command(PIKACommandType.ENABLE)

    def stop(self) -> None:
        self._shutdown_event.set()
        self._thread.join()
        self._send_command(PIKACommandType.DISABLE)
        self.serial.close()

    def _get_data(self) -> None:
        while not self._shutdown_event.is_set():
            try:
                n_in_waiting = self.serial.in_waiting
                if n_in_waiting <= 0:
                    continue
                data = self.serial.read(size=1)
                if data:
                    self._buffer += data.decode("utf-8", errors="ignore")
                    json_data = self._find_json_data()
                    if json_data:
                        self._motor_data = json_data["motor"]
                        self._motor_status = json_data["motorstatus"]
            except Exception as e:
                logger.error("Error getting data: {}", e)

    def get_state(self) -> float:
        angle = float(self._motor_data["Position"])
        gripper_width = (self.get_distance(angle) - self.get_distance(0)) * 2
        return gripper_width

    def set_motor_angle(self, rad: float) -> None:
        # make sure the angle is non-negative
        if rad < 0:
            rad = 0
            logger.warning("motor angle cannot be negative, set to 0")

        # if the current is greater than -2100, control normally
        if float(self._motor_data["Current"]) > -2100:
            self.rad = rad
            self._send_command(PIKACommandType.POSITION_CTRL, rad)
        # if the current is less than -10000, exit with error
        elif float(self._motor_data["Current"]) < -10000:
            logger.error(
                "motor is over current, please check the gripper motor status, if the red light is on, please power off and restart"
            )
        else:
            logger.warning(
                "motor is not in normal control mode, current={}", self._motor_data["Current"]
            )

    def set_motor_torque(self, current: float = 2.0) -> bool:
        """Set the motor current, which affects the torque of the gripper.

        current (float): current value, unit is A. range: 0~3A
        default: 2A, which can support 20N force.
        """
        # make sure the current is within [0, 3]
        if current < 0:
            current = 0
            logger.warning("motor current cannot be negative, set to 0")
        elif current > 3:
            current = 3
            logger.warning("motor current cannot be greater than 3A, set to 3A")

        return self._send_command(PIKACommandType.CURRENT, current)

    def gripper_control(self, gripper_width: float) -> None:
        gripper_width = max(0, min(gripper_width, 0.095))

        target_distance = gripper_width / 2 + self._distance_0
        # define the search range
        # assume the angle is between 0 and (180.0 - 43.99) / 180.0 * math.pi radians
        low_angle = 0.0
        high_angle = (180.0 - 43.99) / 180.0 * math.pi  # Approximately 1.99 radians

        min_gripper_distance_at_low_angle = (self.get_distance(low_angle) - self._distance_0) * 2
        max_gripper_distance_at_high_angle = (self.get_distance(high_angle) - self._distance_0) * 2

        # Ensure target_gripper_distance_mm is within the valid range
        if not (
            min_gripper_distance_at_low_angle <= gripper_width <= max_gripper_distance_at_high_angle
        ):
            logger.error(
                f"target gripper width {gripper_width:.2f} m exceed the range [{min_gripper_distance_at_low_angle:.2f}, {max_gripper_distance_at_high_angle:.2f}] m"
            )
            return

        tolerance = 0.00001  # gripper width tolerance (m)
        angle_tolerance = 0.00001  # angle tolerance (rad)
        max_iterations = 1000

        found_angle = None

        for _ in range(max_iterations):
            mid_angle = (low_angle + high_angle) / 2
            current_distance = self.get_distance(mid_angle)

            # Check if the current_width_mm is close enough to target_width_mm
            if abs(current_distance - target_distance) < tolerance:
                found_angle = mid_angle
                break

            # Since get_distance(angle) increases with angle:
            if current_distance < target_distance:
                low_angle = mid_angle
            else:
                high_angle = mid_angle

            # If the angle range becomes too small, stop
            if (high_angle - low_angle) < angle_tolerance:
                found_angle = mid_angle  # Use the midpoint as the best approximation
                break

        if found_angle is not None:
            self.set_motor_angle(found_angle)
        else:
            logger.error(f"cannot find the angle for target gripper width {gripper_width:.2f} m")

    def set_zero(self) -> None:
        self._send_command(PIKACommandType.SET_ZERO)


def control_loop(client: PikaGripperClient, control_frequency: int) -> None:
    control_interval = 1 / control_frequency
    start_time = time.time()

    while True:
        # Create a linear traj looping between 0 to 0.095
        traj = np.linspace(0, 0.095, 200)
        for width in traj:
            client.gripper_control(width)
            current_time = time.time()
            if current_time - start_time >= control_interval:
                start_time = current_time
                client.gripper_control(0.095)
            else:
                # Wait for the control interval
                time.sleep(0.001)
        # From 0.095 to 0
        traj = np.linspace(0.095, 0, 200)
        for width in traj:
            client.gripper_control(width)
            current_time = time.time()
            if current_time - start_time >= control_interval:
                start_time = current_time
                client.gripper_control(0)
            else:
                # Wait for the control interval
                time.sleep(0.001)


if __name__ == "__main__":
    client = PikaGripperClient()
    client.start()
    # Start the control loop
    control_frequency = 100
    threading.Thread(target=control_loop, args=(client, control_frequency)).start()
    # Test the frequency of pika reading
    
    control_interval = 1 / control_frequency
    start_time = time.time()
    while True:
        current_time = time.time()
        if current_time - start_time >= control_interval:
            start_time = current_time
            client.get_state()
        else:
            time.sleep(0.001)