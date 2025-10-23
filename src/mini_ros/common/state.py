
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
    timestamp: float = 0
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
    timestamp: float = 0.0
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
class CameraData:
    """
    Camera data
    """
    # 1: Timestamp at camera chip
    timestamp: float = 0
    # 1: Timestamp recv by system
    timestamp_in_system: float = 0
    # (H, W, 3): (H is the height, W is the width)
    color_image: np.ndarray | bytes = None
    # (H, W): (H is the height, W is the width)
    depth_image: np.ndarray | bytes = None
    # meta info
    width: int = 0
    height: int = 0


@dataclass
class TimedData:
    """
    Timed data
    """
    timestamp: int = 0
    code: str = "normal"  # Code for the data
    data: Any = None     # Data


@dataclass
class FrameTime:
    timestamp_ns: int
    timestamp_recv_ns: int


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
    CONTROL = 1  # Enable the moving of robot
    PAUSE = 2  # Disable the moving of robot
    DRAG = 3     # Enter the drag mode, when you can drag the robot


##########################################
### Back compatibility with RDC ##########
##########################################

@functools.total_ordering
class CommanderState(OrderedEnum):
    """
    The state of the commander.
    To back-compatibility with RDC. Remove this after the transition is complete.
    """

    DEAD = 0
    INIT = 1         # [New] -> INIT state
    ACTIVE = 2       # [New] -> ENABLE state
    ALIGN = 3        # [New] -> ALIGN state
    RECORD = 4       # [New] -> RECORD state
    RESTORE = 5      
    END = 6
    FAULT = 7
    STOPPED = 8       # [New] -> STOP state
    WAITING = 9

    # Unique to Autopilot
    AP_INFERENCE = 10
    AP_ALIGN = 11
    AP_PRE_TELEOP = 12
    AP_TELEOP = 13
    AP_READY = 14

    # Unique to Some-robots
    EMERGENCY = 15
    DRAG = 16         # [New] -> DRAG state
    REBOOT = 17

    LOSE_CONNECTION = 18