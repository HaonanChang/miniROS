
"""
List all kinds of state from mini_ros.
"""

from typing import Any
import functools
from dataclasses import dataclass, field
import numpy as np
from mini_ros.utils.ordered_enum import OrderedEnum


@functools.total_ordering
class InputDeviceState(OrderedEnum):
    """
    State for recording devices.
    """
    INIT = 0
    OPEN = 1   
    RUNNING = 2
    STOPPED = 3


@functools.total_ordering
class RobotDeviceState(OrderedEnum):
    """
    State for robot devices.
    """
    INIT = 0
    OPEN = 1   
    RUNNING = 2
    STOPPED = 3


@dataclass
class RobotState:
    """
    Robot state
    """
    # 1: Timestamp
    timestamp: int = 0
    # (N,): (N is the number of joints)
    joint_positions: list[float] = field(default_factory=list)
    # (N,): (N is the number of joints)
    joint_velocities: list[float] = field(default_factory=list)
    # (N,): (N is the number of joints)
    joint_efforts: list[float] = field(default_factory=list)
    # (N, ): (N is the number of grippers)
    joint_currents: list[float] = field(default_factory=list)
    # (M, 6): (x, y, z, qw, qx, qy, qz), M is the number of end effectors
    end_effector_positions: list[float] = field(default_factory=list)
    # (7,): (x, y, z, qw, qx, qy, qz)
    base_pose: list[float] = field(default_factory=list)
    # (6,): (x, y, z, roll, pitch, yaw)
    base_velocity: list[float] = field(default_factory=list)


@dataclass
class RobotAction:
    """
    Robot action
    """
    # 1: Timestamp
    timestamp: int
    # (N,): (N is the number of joints)
    # Joint-space Position Control
    joint_cmds: list[float] = field(default_factory=list)
    # (M, 6): (x, y, z, qw, qx, qy, qz), M is the number of end effectors
    # End-effector Position Control
    end_effector_cmds: list[float] = field(default_factory=list)
    # (7,): (x, y, z, qw, qx, qy, qz)
    # Base Pose Control
    base_pose_cmds: list[float] = field(default_factory=list)
    # (6,): (x, y, z, roll, pitch, yaw)
    # Base Velocity Control
    base_velocity_cmds: list[float] = field(default_factory=list)
    

@dataclass
class TimedData:
    """
    Timed data
    """
    timestamp: int
    data: Any
