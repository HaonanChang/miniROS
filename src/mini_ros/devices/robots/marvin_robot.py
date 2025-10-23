import numpy as np
import serial
import threading
import time
import struct
import re
import math
import glob
import json
import queue
import os
import copy
from dataclasses import dataclass, field
from enum import Enum
from loguru import logger
from mini_ros.common.device import Robot, RobotState, RobotAction, Recorder
from mini_ros.common.error import RobotExecuteError
from mini_ros.utils.time_util import TimeUtil
from mini_ros.utils.io_util import IOUtil

root_path = os.path.dirname(os.path.abspath(__file__))
root_path = os.path.join(root_path, "../../../..")
import sys
sys.path.append(root_path)

from third_party.marvin_sdk.fx_robot import Marvin_Robot
from third_party.marvin_sdk.structure_data import DCSS


@dataclass
class MarvinRobotConfig:
    port: str = "192.168.1.190"
    baudrate: int = 115200
    timeout: float = 1.0
    init_timeout: float = 10.0
    acc_ratio: int = 10
    vel_ratio: int = 10
    control_mode: str = "impedance"
    # # New end-effector
    # dynamic_params: list = field(default_factory=lambda: [
    #     [0.715, -6.0, 43.0, 87.0, 0.001, 0.0, 0.0, -0.002, 0.0, 0.005],
    #     [0.715, 6.0, -43.0, 87.0, 0.001, 0.0, 0.0, -0.002, 0.0, 0.005]
    # ])
    # Old end-effector
    dynamic_params: list = field(default_factory=lambda: [
        [0.92, -3.365, 35.458, 75.8, -0.00106, 0.0, 0.0, -0.0054, 0.0, 0.0042],
        [0.92, 7.218, -39.120, 79.375, -0.000487, 0.0,  0.0, -0.00105, 0.0, 0.000997]
    ])
    default_joint_pos: list = field(default_factory=lambda: [
        [90.0, -70.0, -90.0, -90.0, 0.0, 0.0, 0.0],
        [-90.0, -70.0, 90.0, -90.0, 0.0, 0.0, 0.0]
    ])
    mute_skd_log: bool = True
    is_new_version: bool = True


class MarvinRobot(Robot):
    """
    Marvin robot interface.
    """
    name: str = "marvin"

    def __init__(self, config: MarvinRobotConfig):
        self.port = config.port
        self.baudrate = config.baudrate
        self.timeout = config.timeout
        self.acc_ratio = config.acc_ratio
        self.vel_ratio = config.vel_ratio
        self.control_mode = config.control_mode
        self.dynamic_params = config.dynamic_params
        self.mute_skd_log = config.mute_skd_log
        self.is_new_version = config.is_new_version
        self.init_timeout = config.init_timeout
        self.mode = "init"
        self.recorder: Recorder = None
        # Active: Can be controlled
        self._active_event = threading.Event()
        # Connect: Connected to the robot
        self._connect_event = threading.Event()

    def initialize(self):
        self.robot = Marvin_Robot()
        self.robot.connect(self.port)
        start_time = TimeUtil.now()
        while (TimeUtil.now() - start_time).total_seconds() < self.init_timeout:
            state = self.get_state()
            if state is not None and self.is_state_valid(state):
                break
            time.sleep(0.1)
        if state is None or not self.is_state_valid(state):
            logger.error(f"Failed to initialize Marvin robot in {self.init_timeout} seconds")
            return False
        logger.info(f"Initialized Marvin robot in {TimeUtil.get_elapsed_time_ms(start_time)} ms")
        self._connect_event.set()
        return True

    def is_state_valid(self, state: RobotState) -> bool:
        """
        Check if the Marvin's robot state is valid.
        If not connected, the marvin's joint positions should be all zeros.
        """
        return np.all(state.joint_positions != 0)

    def is_active(self) -> bool:
        return self._active_event.is_set()
    
    def is_alive(self) -> bool:
        return self._connect_event.is_set()

    def get_state(self, timeout: float = 1.0, is_record: bool = False) -> RobotState:
        """
        timestamp should be the recv time.
        """
        outputs = self.robot.subscribe(DCSS())["outputs"]
        timestamp = TimeUtil.now().timestamp()
        robot_state = RobotState(
            timestamp=timestamp,
            joint_positions=np.concatenate([outputs[0]["fb_joint_pos"], outputs[1]["fb_joint_pos"]]),
            joint_velocities=np.concatenate([outputs[0]["fb_joint_vel"], outputs[1]["fb_joint_vel"]]),
            # joint_efforts=np.concatenate([outputs[0]["fb_joint_torque"], outputs[1]["fb_joint_torque"]]),
            end_effector_positions=np.concatenate([outputs[0]["fb_joint_posE"], outputs[1]["fb_joint_posE"]]),
        )
        if is_record and self.recorder is not None:
            self.recorder.put(robot_state, "robot_state")
        return robot_state

    def apply_action(self, action: RobotAction, is_record: bool = False) -> RobotAction:
        """
        Action is in degree.
        Dim (14,): The first 7 are left arm, the last 7 are right arm.
        Example:
        action = RobotAction(timestamp=0, joint_cmds=[90.0, -70.0, -90.0, -110.0, 90.0, 0.0, 0.0, -90.0, -70.0, 90.0, -110.0, -90.0, 0.0, 0.0])
        self.apply_action(action)
        """
        if self.mute_skd_log:
            IOUtil.mute()
        self.robot.clear_set()
        if self.control_mode == "impedance" or self.control_mode == "position":
            left_action = action.joint_cmds[:7]
            right_action = action.joint_cmds[7:]
            left_action = self._joint_project_clip(left_action, self.is_new_version)
            right_action = self._joint_project_clip(right_action, self.is_new_version)
            self.robot.set_joint_cmd_pose(
                "A", joints=left_action
            )
            self.robot.set_joint_cmd_pose(
                "B", joints=right_action
            )
        else:
            raise ValueError(f"Invalid control mode: {self.control_mode}")
        timestamp = TimeUtil.now().timestamp()
        self.robot.send_cmd()
        action = copy.deepcopy(action)
        action.timestamp = timestamp
        if self.mute_skd_log:
            IOUtil.restore()
        if is_record and self.recorder is not None:
            self.recorder.put(action, "robot_action")
        return action

    def start(self, episode_name: str):
        """
        NOTE: Start is a blocking call.
        """
        if not self.is_alive():
            # Can't be double started
            logger.warning("Marvin robot is not connected, can't be started")
            return
        self._launch_robot()
        if self.recorder is not None:
            self.recorder.start(episode_name)
        # Set active event
        self._active_event.set()

    def _launch_robot(self):
        if self.mute_skd_log:
            IOUtil.mute()
        # Clear error
        self.robot.clear_error("A")
        self.robot.clear_error("B")
        time.sleep(0.25)

        # Set dynamic params
        self.robot.clear_set()
        self.robot.set_tool(arm="A", kineParams=[0, 0, 0, 0, 0, 0], dynamicParams=self.dynamic_params[0])
        self.robot.set_tool(arm="B", kineParams=[0, 0, 0, 0, 0, 0], dynamicParams=self.dynamic_params[1])
        self.robot.send_cmd()
        time.sleep(0.25)

        if self.mute_skd_log:
            IOUtil.restore()

        # Set control mode
        self.robot.clear_set()
        if self.control_mode == "impedance":
            self.switch_mode("impedance")
        elif self.control_mode == "position":
            self.switch_mode("position")
        else:
            raise ValueError(f"Invalid control mode: {self.control_mode}")

        if self.mute_skd_log:
            IOUtil.mute()
        # Set velocity and acceleration
        self.robot.set_vel_acc("A", self.vel_ratio, self.acc_ratio)
        self.robot.set_vel_acc("B", self.vel_ratio, self.acc_ratio)
        self.robot.send_cmd()

        time.sleep(0.25)
        if self.mute_skd_log:
            IOUtil.restore()
        
    def stop(self):
        """
        NOTE: Stop is a blocking call.
        """
        if not self.is_alive():
            # Can't be double stopped
            logger.warning("Marvin robot is not connected, can't be stopped")
            return
        self._active_event.clear()
        self._connect_event.clear()
        self._pause_robot()
        self.robot.release_robot()
        if self.recorder is not None:
            self.recorder.stop()

    def pause(self):
        """
        NOTE: Pause is a blocking call.
        """
        if not self.is_active():
            # Can't be double paused
            logger.warning("Marvin robot is not active, can't be paused")
            return
        self._active_event.clear()
        logger.info(f"Pausing Marvin robot: {self.name}: Active: {self.is_active()}, Alive: {self.is_alive()}")
        self._pause_robot()
        if self.recorder is not None:
            self.recorder.save()

    def _pause_robot(self):
        if self.mute_skd_log:
            IOUtil.mute()
        # Set robot to stop state
        self.robot.clear_set()
        self.robot.set_state("A", 0)
        self.robot.set_state("B", 0)
        self.robot.send_cmd()
        if self.mute_skd_log:
            IOUtil.restore()
        time.sleep(0.25)
        self.mode = "init"  # Set mode back to init after pause

    def reboot(self):
        logger.info("Rebooting Marvin robot...")
        self._pause_robot()
        self._launch_robot()
        # Set active event
        self._active_event.set()
        logger.info("Marvin robot rebooted")

    def set_safe_speed(self):
        """
        Set robot to safe mode.
        """
        if self.mute_skd_log:
            IOUtil.mute()
        self.robot.clear_set()
        self.robot.set_vel_acc("A", 10, 10)
        self.robot.set_vel_acc("B", 10, 10)
        self.robot.send_cmd()
        if self.mute_skd_log:
            IOUtil.restore()
        time.sleep(0.25)

    def set_normal_speed(self):
        """
        Set robot to normal mode.
        """
        if self.mute_skd_log:
            IOUtil.mute()
        self.robot.clear_set()
        self.robot.set_vel_acc("A", self.vel_ratio, self.acc_ratio)
        self.robot.set_vel_acc("B", self.vel_ratio, self.acc_ratio)
        self.robot.send_cmd()
        if self.mute_skd_log:
            IOUtil.restore()
        time.sleep(0.25)

    def switch_mode(self, mode: str):
        """
        Swith between different modes:
         - init: mode robot is not initialized.
         - drag: mode robot can be drag.
         - impedance: mode robot can be controlled.
         - position: mode robot can be controlled.
        """
        if mode == self.mode:
            return

        if mode == "drag":
            if self.mode != "impedance":
                # Drag mode needs to be set impedance mode first.
                self._set_impedance_mode()
            self._set_drag_mode()
        elif mode == "impedance":
            if self.mode == "drag":
                self._unset_drag_mode()
            self._set_impedance_mode()
        elif mode == "position":
            if self.mode == "drag":
                self._unset_drag_mode()
            self._set_position_mode()
        else:
            raise ValueError(f"Invalid mode: {mode}")

    def _set_drag_mode(self):
        """
        Set robot to drag mode.
        """
        if self.mute_skd_log:
            IOUtil.mute()
        self.robot.clear_set()
        self.robot.set_joint_kd_params(
            arm="A", K=[2, 2, 2, 1.6, 1, 1, 1], D=[0.4, 0.4, 0.4, 0.3, 0.2, 0.2, 0.2]
        )
        self.robot.set_joint_kd_params(
            arm="B", K=[2, 2, 2, 1.6, 1, 1, 1], D=[0.4, 0.4, 0.4, 0.3, 0.2, 0.2, 0.2]
        )
        self.robot.send_cmd()
        time.sleep(0.25)

        self.robot.clear_set()
        self.robot.set_drag_space(arm="A", dgType=1)
        self.robot.set_drag_space(arm="B", dgType=1)
        self.robot.send_cmd()
        time.sleep(0.25)

        if self.mute_skd_log:
            IOUtil.restore()
        self.mode = "drag"

    def _unset_drag_mode(self):
        """
        Unset robot from drag mode.
        """
        if self.mute_skd_log:
            IOUtil.mute()
        self.robot.clear_set()
        self.robot.set_drag_space(arm="A", dgType=0)
        self.robot.set_drag_space(arm="B", dgType=0)
        self.robot.send_cmd()
        time.sleep(0.25)

        self.robot.clear_set()
        self.robot.set_joint_kd_params(
            arm="A", K=[5, 5, 5, 2.5, 2.5, 2.5, 2.5], D=[0.5, 0.5, 0.5, 0.4, 0.2, 0.2, 0.2]
        )
        self.robot.set_joint_kd_params(
            arm="B", K=[5, 5, 5, 2.5, 2.5, 2.5, 2.5], D=[0.5, 0.5, 0.5, 0.4, 0.2, 0.2, 0.2]
        )
        self.robot.send_cmd()
        time.sleep(0.25)

        if self.mute_skd_log:
            IOUtil.restore()
        self.mode = "impedance"

    def _set_impedance_mode(self):
        """
        Set robot to impedance mode.
        """
        if self.mute_skd_log:
            IOUtil.mute()
        self.robot.clear_set()
        self.robot.set_state("A", 3)
        self.robot.set_state("B", 3)
        self.robot.send_cmd()
        time.sleep(0.25)
        self.robot.clear_set()
        self.robot.set_joint_kd_params(
            arm="A", K=[5, 5, 5, 2.5, 2.5, 2.5, 2.5], D=[0.5, 0.5, 0.5, 0.4, 0.2, 0.2, 0.2]
        )
        self.robot.set_joint_kd_params(
            arm="B", K=[5, 5, 5, 2.5, 2.5, 2.5, 2.5], D=[0.5, 0.5, 0.5, 0.4, 0.2, 0.2, 0.2]
        )
        self.robot.send_cmd()
        time.sleep(0.25)
        if self.mute_skd_log:
            IOUtil.restore()
        self.mode = "impedance"


    def _set_position_mode(self):
        """
        Set robot to position mode.
        """
        if self.mute_skd_log:
            IOUtil.mute()
        self.robot.clear_set()
        self.robot.set_state("A", 1)
        self.robot.set_state("B", 1)
        self.robot.send_cmd()
        time.sleep(0.25)

        self.robot.clear_set()
        self.robot.set_joint_kd_params(
            arm="A", K=[5, 5, 5, 2.5, 2.5, 2.5, 2.5], D=[0.5, 0.5, 0.5, 0.4, 0.2, 0.2, 0.2]
        )
        self.robot.set_joint_kd_params(
            arm="B", K=[5, 5, 5, 2.5, 2.5, 2.5, 2.5], D=[0.5, 0.5, 0.5, 0.4, 0.2, 0.2, 0.2]
        )
        self.robot.send_cmd()
        time.sleep(0.25)
        if self.mute_skd_log:
            IOUtil.restore()
        self.mode = "position"

    @property
    def num_dof(self) -> int:
        """
        Marvin robot is dual arm robot, 14 dof.
        """
        return 14

    @property
    def max_control_freq(self) -> int:
        return 60

    @property
    def max_read_freq(self) -> int:
        return 200

    def _joint_project_clip(self, joints_rad, is_new_version: bool = False):
        """
        Project joints on the joint boundary. Dependend on the version, we may have different methods.
        """
        ## Perform j6-j7 joint clipping
        j6 = joints_rad[5]
        j7 = joints_rad[6]
        j6 = np.clip(j6, -np.pi / 3, np.pi / 3)
        j7 = np.clip(j7, -np.pi / 2, np.pi / 2)
        j6_sign = 1 if j6 >= 0 else -1
        j7_sign = 1 if j7 >= 0 else -1
        X0 = np.abs(j6)
        Y0 = np.abs(j7)

        # Linear range: Y = AX + B:
        B = 78 / 180 * np.pi if not is_new_version else 109 / 180 * np.pi
        A = -1

        if Y0 > (A * X0 + B):
            # need projection
            X = (A * Y0 + X0 - A * B) / (A**2 + 1)
            Y = A * X + B
            if X < 0:
                X = 0
                Y = B
            elif Y < 0:
                X = -B / A
                Y = 0
            elif X < 0 and Y < 0:
                raise ValueError("Theoretically impossible.")
            j6 = j6_sign * X
            j7 = j7_sign * Y
        else:
            j6 = j6_sign * X0
            j7 = j7_sign * Y0
        joints_rad[5] = j6
        joints_rad[6] = j7
        return joints_rad