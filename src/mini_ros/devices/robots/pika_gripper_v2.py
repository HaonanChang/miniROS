import threading
import numpy as np
import serial
import time
import struct
import re
import math
import glob
import json
from dataclasses import dataclass
from enum import Enum
from loguru import logger
from mini_ros.common.device import Robot
from mini_ros.common.error import RobotExecuteError
from mini_ros.utils.time_util import TimeUtil
from mini_ros.common.state import RobotState, RobotAction
import copy

PIKA_GRIPPER_WIDTH = 0.09  # m


class PIKACommandType(Enum):
    DISABLE = 10
    ENABLE = 11
    SET_ZERO = 12
    CURRENT = 15
    EFFORT_CTRL = 20
    VELOCITY_CTRL = 21
    POSITION_CTRL = 22


@dataclass
class PikaGripperConfig:
    port: str | None = None
    baudrate: int = 460800
    max_motor_torque_current: float = 2.0
    timeout: float = 1.0


class PikaGripper(Robot):
    """
    Pika gripper interface.
    """
    def __init__(
        self,
        config: PikaGripperConfig = PikaGripperConfig(),
    ) -> None:
        self.port = config.port
        self.baudrate = config.baudrate
        self.timeout = config.timeout
        self.max_motor_torque_current = config.max_motor_torque_current
        self.data_lock = threading.Lock()
        self.polling_rate = 100
        if self.port is None:
            # Find the first available port
            ports = glob.glob("/dev/ttyUSB*")
            if not ports:
                raise RuntimeError("No serial ports found.")
            if len(ports) > 1:
                logger.info("Multiple serial ports found. Using the first one.")
            self.port = ports[0]

        self.serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
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
        self.set_motor_torque(self.max_motor_torque_current)

    def _send_command(self, command_type: PIKACommandType, value: float = 0.0) -> bool:
        try:
            data = bytearray()
            data.append(int(command_type))  # command type

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
        self._send_command(PIKACommandType.ENABLE.value)

    def stop(self) -> None:
        self._shutdown_event.set()
        self._thread.join()
        self._send_command(PIKACommandType.DISABLE.value)
        self.serial.close()

    def is_active(self):
        return not self._shutdown_event.is_set()

    def _get_data(self) -> None:
        while self.is_active():
            start_time = time.time()
            try:
                n_in_waiting = self.serial.in_waiting
                logger.info(f"n_in_waiting: {n_in_waiting}")
                if n_in_waiting <= 0:
                    time.sleep(0.001)
                    continue
                data = self.serial.read(size=4)
                if data:
                    self._buffer += data.decode("utf-8", errors="ignore")
                    print(data.decode("utf-8", errors="ignore"))
                    json_data = self._find_json_data()
                    if json_data:
                        with self.data_lock:
                            self._motor_data = json_data["motor"]
                            self._motor_status = json_data["motorstatus"]
                            logger.info(f"writting data: {self._motor_data['Position']}")
                        # Clear buffer
                        if len(self._buffer) > 2000:
                            self._buffer = ""
            except Exception as e:
                logger.error("Error getting data: {}", e)
            # # Hand over control
            # current_time = time.time()
            # if current_time - start_time >= 1 / self.polling_rate:
            #     start_time = current_time
            #     self.robot.apply_action(RobotAction(timestamp=TimeUtil.now().timestamp(), joint_cmds=[0]))
            # else:
            #     # Wait for the control interval
            #     time.sleep(0.002)

    def get_state(self) -> RobotState:
        with self.data_lock:
            angle = float(self._motor_data["Position"])
            speed = float(self._motor_data["Speed"])
            current = float(self._motor_data["Current"])
        gripper_width = (self.get_distance(angle) - self.get_distance(0)) * 2
        logger.info(f"reading data: {angle}, gripper_width: {gripper_width}")
        # Get timestamp
        timestamp = TimeUtil.now().timestamp()
        return RobotState(
            joint_positions=[gripper_width / PIKA_GRIPPER_WIDTH],
            joint_velocities=[speed],
            joint_currents=[current],
            timestamp=timestamp,
        )

    def set_motor_angle(self, rad: float) -> None:
        with self.data_lock:
            current = self._motor_data["Current"]
        # make sure the angle is non-negative
        if rad < 0:
            rad = 0
            logger.warning("motor angle cannot be negative, set to 0")

        # if the current is greater than -2100, control normally
        if float(current) > -2100:
            self.rad = rad
            self._send_command(PIKACommandType.POSITION_CTRL.value, rad)
        # if the current is less than -10000, exit with error
        elif float(current) < -10000:
            logger.error(
                "motor is over current, please check the gripper motor status, if the red light is on, please power off and restart"
            )
        else:
            logger.warning(
                "motor is not in normal control mode, current={}", current
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

        return self._send_command(PIKACommandType.CURRENT.value, current)

    def apply_action(self, action: RobotAction) -> RobotAction:
        gripper_width = action.joint_cmds[0]  # (0~1)
        gripper_width = np.clip(gripper_width, 0, 1) * PIKA_GRIPPER_WIDTH

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
            # Log the exact timestamp of the action
            timestamp = TimeUtil.now().timestamp()
            robot_action = copy.deepcopy(action)
            robot_action.timestamp = timestamp
            logger.info(f"Applying action {gripper_width / PIKA_GRIPPER_WIDTH}")
            return robot_action
        else:
            logger.error(f"cannot find the angle for target gripper width {gripper_width:.2f} m")
            raise RobotExecuteError(f"cannot find the angle for target gripper width {gripper_width:.2f} m", code="GRIPPER_ANGLE_NOT_FOUND")
        
    def set_zero(self) -> None:
        self._send_command(PIKACommandType.SET_ZERO.value)

    def reboot(self):
        pass

    def initialize(self, driver_config):
        pass