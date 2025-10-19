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
import copy
from dataclasses import dataclass
from enum import Enum
from loguru import logger
from mini_ros.common.device import Robot, RobotState, RobotAction
from mini_ros.common.error import RobotExecuteError
from mini_ros.utils.time_util import TimeUtil

from third_party.marvin_sdk.fx_robot import Marvin_Robot
from third_party.marvin_sdk.structure_data import DCSS
# from src.inputs.marvin_state_output import MarvinStateOutput


@dataclass
class MarvinRobotConfig:
    port: str = "192.168.1.190"
    baudrate: int = 115200
    timeout: float = 1.0
    acc_ratio: int = 100
    vel_ratio: int = 100
    control_mode: str = "impedance"
    dynamic_params: list = [[0.715, -6.0, 43.0, 87.0, 0.001, 0.0, 0.0, -0.002, 0.0, 0.005],
                            [0.715, 6.0, -43.0, 87.0, 0.001, 0.0, 0.0, -0.002, 0.0, 0.005]]
    default_joint_pos: list = [[90.0, -70.0, -90.0, -90.0, 0.0, 0.0, 0.0],
                               [-90.0, -70.0, 90.0, -90.0, 0.0, 0.0, 0.0]]


class MarvinRobot(Robot):
    """
    Marvin robot interface.
    Because python has GIL, so actually, you don't need thread Lock.
    """
    def __init__(self, config: MarvinRobotConfig):
        self.port = config.port
        self.baudrate = config.baudrate
        self.timeout = config.timeout
        self.acc_ratio = config.acc_ratio
        self.vel_ratio = config.vel_ratio
        self.control_mode = config.control_mode
        self.dynamic_params = config.dynamic_params
        self.mode = "init"

    def initialize(self):
        self.robot = Marvin_Robot()
        self.robot.connect(self.port)
        time.sleep(0.25)

    def start(self):
        self._launch_robot()

    def _launch_robot(self):
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

        # Set control mode
        self.robot.clear_set()
        if self.control_mode == "impedance":
            self.switch_mode("impedance")
        elif self.control_mode == "position":
            self.switch_mode("position")
        else:
            raise ValueError(f"Invalid control mode: {self.control_mode}")

        # Set velocity and acceleration
        self.robot.set_vel_acc("A", self.vel_ratio, self.acc_ratio)
        self.robot.set_vel_acc("B", self.vel_ratio, self.acc_ratio)
        self.robot.send_cmd()
        time.sleep(0.25)

    def get_state(self, timeout: float = 1.0) -> RobotState:
        outputs = self.robot.subscribe(DCSS())["outputs"]
        return RobotState(
            joint_positions=np.concatenate([outputs[0]["fb_joint_pos"], outputs[1]["fb_joint_pos"]]),
            joint_velocities=np.concatenate([outputs[0]["fb_joint_vel"], outputs[1]["fb_joint_vel"]]),
            joint_efforts=np.concatenate([outputs[0]["fb_joint_torque"], outputs[1]["fb_joint_torque"]]),
            # end_effector_positions=np.concatenate([outputs[0]["fb_ee_pos"], outputs[1]["fb_ee_pos"]]),
        )

    def apply_action(self, action: RobotAction) -> RobotAction:
        """
        Action is in degree.
        """
        self.robot.clear_set()
        if self.control_mode == "impedance" or self.control_mode == "position":
            self.robot.set_joint_cmd_pose(
                "A", joints=action.joint_cmds[:7]
            )
            self.robot.set_joint_cmd_pose(
                "B", joints=action.joint_cmds[7:]
            )
        else:
            raise ValueError(f"Invalid control mode: {self.control_mode}")
        timestamp = TimeUtil.now().timestamp()
        self.robot.send_cmd()
        action = copy.deepcopy(action)
        action.timestamp = timestamp
        return action

    def stop(self):
        self._pause_robot()
        self.robot.release_robot()

    def _pause_robot(self):
        # Set robot to stop state
        self.robot.clear_set()
        self.robot.set_state("A", 0)
        self.robot.set_state("B", 0)
        self.robot.send_cmd()
        time.sleep(0.25)
        self.mode = "init"  # Set mode back to init after pause

    def reboot(self):
        logger.info("Rebooting Marvin robot...")
        self._pause_robot()
        self._launch_robot()
        logger.info("Marvin robot rebooted")

    def set_safe_speed(self):
        """
        Set robot to safe mode.
        """
        self.robot.clear_set()
        self.robot.set_vel_acc("A", 10, 10)
        self.robot.set_vel_acc("B", 10, 10)
        self.robot.send_cmd()
        time.sleep(0.25)

    def set_normal_speed(self):
        """
        Set robot to normal mode.
        """
        self.robot.clear_set()
        self.robot.set_vel_acc("A", self.vel_ratio, self.acc_ratio)
        self.robot.set_vel_acc("B", self.vel_ratio, self.acc_ratio)
        self.robot.send_cmd()
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
        self.mode = "drag"

    def _unset_drag_mode(self):
        """
        Unset robot from drag mode.
        """
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
        self.mode = "impedance"

    def _set_impedance_mode(self):
        """
        Set robot to impedance mode.
        """
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
        self.mode = "impedance"


    def _set_position_mode(self):
        """
        Set robot to position mode.
        """
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
        self.mode = "position"

    @property
    def num_dof(self) -> int:
        """
        Marvin robot is dual arm robot, 14 dof.
        """
        return 14