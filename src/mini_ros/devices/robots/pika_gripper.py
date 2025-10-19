from sqlite3 import Time
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
        config: PikaGripperConfig,
    ) -> None:
        self.port = config.port
        self.baudrate = config.baudrate
        self.timeout = config.timeout
        self.max_motor_torque_current = config.max_motor_torque_current

    def initialize(self):
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
        self._distance_0 = self._get_distance(0)
        self._overflow_counter = 0
        self._overflow_threshold = 100
        self._set_motor_torque(self.max_motor_torque_current)

    def start(self) -> None:
        try:
            self._send_command(PIKACommandType.ENABLE)
            return True
        except Exception as e:
            logger.error("Error starting Pika gripper: {}", e)
            return False

    def stop(self) -> None:
        try:
            self._send_command(PIKACommandType.DISABLE)
            self.serial.close()
            return True
        except Exception as e:
            logger.error("Error stopping Pika gripper: {}", e)
            return False
    
    def reboot(self) -> None:
        logger.info("Rebooting Pika gripper...")
        self.stop()
        self.start()
        logger.info("Pika gripper rebooted")

    def get_state(self, timeout: float = 1.0) -> RobotState:
        """
        Get state return a value between 0 and 1.
        """
        start_time = TimeUtil.now()
        while True:
            try:
                if TimeUtil.get_elapsed_time(start_time).total_seconds() > timeout:
                    raise TimeoutError("Timeout getting state")
                data = self.read_data()
                if data:
                    self._buffer += data.decode("utf-8", errors="ignore")
                    json_data = self._find_json()
                    if json_data:
                        motor_data = json_data["motor"]
                        motor_status = json_data["motorstatus"]
                        timestamp = TimeUtil.now().timestamp()
                        break
                    # Clear the buffer if it is too long
                    if len(self._buffer) > 2000:
                        self._buffer = ""
                # Handover control
                time.sleep(0.001)
            except Exception as e:
                logger.error("Error getting data: {}", e)

        angle = float(motor_data["Position"])
        gripper_width = (self._get_distance(angle) - self._get_distance(0)) * 2
        gripper_width = np.clip(gripper_width, 0, PIKA_GRIPPER_WIDTH)
        gripper_width = gripper_width / PIKA_GRIPPER_WIDTH  # (0~1)
        return RobotState(
            joint_positions=[gripper_width],
            joint_velocities=[float(motor_data["Speed"])],
            joint_currents=[float(motor_data["Current"])],
            timestamp=timestamp,
        )

    def read_data(self):
        """
        Read data from the serial port
        
        Return:
            bytes: the read data
        """
        if not self.is_connected or not self.serial:
            logger.error("Serial port is not connected, cannot read data")
            return b''
        
        try:
            if self.serial.in_waiting > 0:
                return self.serial.read(self.serial.in_waiting)
            return b''
        except serial.SerialException as e:
            logger.error(f"Read data failed: {e}")
            return b''

    def _find_json(self):
        """
        Find the complete JSON object in the buffer
        
        Return:
            dict: the parsed JSON object, if not found, return None
        """
        try:
            # find the start and end position of the JSON object
            start = self._buffer.find('{')
            if start == -1:
                self._buffer = ""
                return None
            
            # use stack to match the bracket
            stack = []
            for i in range(start, len(self._buffer)):
                if self._buffer[i] == '{':
                    stack.append(i)
                elif self._buffer[i] == '}':
                    if stack:
                        stack.pop()
                        if not stack:  # find the complete JSON object
                            json_str = self._buffer[start:i+1]
                            self._buffer = self._buffer[i+1:]
                            
                            # --- key modification: handle the extra comma ---
                            cleaned_json_str = re.sub(r',\s*}', '}', json_str)
                            cleaned_json_str = re.sub(r',\s*\]', ']', cleaned_json_str)
                            
                            return json.loads(cleaned_json_str)
            
            # if the complete JSON object is not found, keep the buffer
            return None
        except json.JSONDecodeError as e:
            logger.error(f"JSON parse error: {e}")
            self._buffer = ""
            return None
        except Exception as e:
            logger.error(f"Json communication error: {e}")
            self._buffer = ""
            return None

    def apply_action(self, action: RobotAction) -> RobotAction:
        """
        Gripper control is input with a value between 0 and 1.
        """
        gripper_width = action.joint_cmds[0]  # (0~1)
        gripper_width = np.clip(gripper_width, 0, 1) * PIKA_GRIPPER_WIDTH

        target_distance = gripper_width / 2 + self._distance_0
        # Define the search range
        # assume the angle is between 0 and (180.0 - 43.99) / 180.0 * math.pi radians
        low_angle = 0.0
        high_angle = (180.0 - 43.99) / 180.0 * math.pi  # Approximately 1.99 radians

        min_gripper_distance_at_low_angle = (self._get_distance(low_angle) - self._distance_0) * 2
        max_gripper_distance_at_high_angle = (self._get_distance(high_angle) - self._distance_0) * 2

        # Ensure target_gripper_distance_mm is within the valid range
        if not (
            min_gripper_distance_at_low_angle <= gripper_width <= max_gripper_distance_at_high_angle
        ):
            logger.error(
                f"target gripper width {gripper_width:.2f} m exceed the range [{min_gripper_distance_at_low_angle:.2f}, {max_gripper_distance_at_high_angle:.2f}] m"
            )
            raise RuntimeError(
                f"target gripper width {gripper_width:.2f} m exceed the range [{min_gripper_distance_at_low_angle:.2f}, {max_gripper_distance_at_high_angle:.2f}] m"
            )

        tolerance = 0.00001  # gripper width tolerance (m)
        angle_tolerance = 0.00001  # angle tolerance (rad)
        max_iterations = 1000

        found_angle = None

        for _ in range(max_iterations):
            mid_angle = (low_angle + high_angle) / 2
            current_distance = self._get_distance(mid_angle)

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
            # Log the exact timestamp of the action
            timestamp = TimeUtil.now().timestamp()
            robot_action = copy.deepcopy(action)
            robot_action.timestamp = timestamp
            self._set_motor_angle(found_angle)
            return robot_action
        else:
            logger.error(f"cannot find the angle for target gripper width {gripper_width:.2f} m")
            raise RobotExecuteError(f"cannot find the angle for target gripper width {gripper_width:.2f} m", code="GRIPPER_ANGLE_NOT_FOUND")

    def _send_command(self, command_type: PIKACommandType, value: float = 0.0) -> bool:
        try:
            data = bytearray()
            data.append(command_type.value)  # command type

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


    def _get_distance(self, angle: float) -> float:
        angle = (180.0 - 43.99) / 180.0 * math.pi - angle
        height = 0.0325 * math.sin(angle)
        width_d = 0.0325 * math.cos(angle)
        width = math.sqrt(0.058**2 - (height - 0.01456) ** 2) + width_d
        return width

    def _set_motor_angle(self, rad: float) -> None:
        # make sure the angle is non-negative
        if rad < 0:
            rad = 0
            logger.warning("motor angle cannot be negative, set to 0")

        # if the current is greater than -2100, control normally
        if float(self._motor_data["Current"]) > -2100:
            self.rad = rad
            self._send_command(PIKACommandType.POSITION_CTRL, rad)
            self._overflow_counter = 0  # Reset the overflow counter
        # if the current is less than -10000, exit with error
        elif float(self._motor_data["Current"]) < -10000:
            logger.error(
                "motor is over current, please check the gripper motor status, if the red light is on, please power off and restart"
            )
            raise RobotExecuteError(
                "motor is over current, please check the gripper motor status, if the red light is on, please power off and restart",
                code="GRIPPER_OVER_CURRENT"
            )
        else:
            self._overflow_counter += 1
            if self._overflow_counter >= self._overflow_threshold:
                logger.error(
                    "motor is over current, please check the gripper motor status, if the red light is on, please power off and restart"
                )
                raise RobotExecuteError(
                    "motor is over current, please check the gripper motor status, if the red light is on, please power off and restart",
                    code="GRIPPER_OVER_CURRENT"
                )
            else:
                logger.warning(
                    "motor is not in normal control mode, current={}", self._motor_data["Current"]
                )

    def _set_motor_torque(self, current: float = 2.0) -> bool:
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

    def _set_zero(self) -> None:
        self._send_command(PIKACommandType.SET_ZERO)