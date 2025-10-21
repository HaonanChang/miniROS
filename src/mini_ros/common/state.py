
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
    timestamp: int = 0
    code: str = "normal"  # Code for the data
    data: Any = None     # Data


##########################################
### Different state for state machine ####
##########################################
@functools.total_ordering
class RDCState(OrderedEnum):
    """
    State for Robot-Data-Collection
    """
    INIT = 0    # Initializing state
    ALIGN = 1   # Aligning robot to gello
    ENABLE = 2  # Enable the moving of robot
    RECORD = 3  # Recording state
    STOP = 4    # Pause the robot (Can interrupt anytime)
    DRAG = 5    # Enter the drag mode, when you can drag the robot


@functools.total_ordering
class RobotNodeState(OrderedEnum):
    """
    State for robot internal state
    """
    INIT = 0    # Initializing state
    ALIGN = 1   # Aligning robot to gello
    ENABLE = 2  # Enable the moving of robot
    DISABLE = 3 # Disable the moving of robot
    DRAG = 4    # Enter the drag mode, when you can drag the robot