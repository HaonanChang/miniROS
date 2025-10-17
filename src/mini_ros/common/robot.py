"""
Robot device:
Robot must implement the following methods:
- start
- reboot
- stop
- reset
- get_state
- set_state
- apply_action
"""

from dataclasses import dataclass, field
import abc
from mini_ros.common.device import Device
import asyncio
from mini_ros.utils.async_util import AsyncUtil


@dataclass
class RobotState:
    """
    Robot state
    """
    # (N,): (N is the number of joints)
    joint_positions: list[float] = field(default_factory=list)
    # (N,): (N is the number of joints)
    joint_velocities: list[float] = field(default_factory=list)
    # (N,): (N is the number of joints)
    joint_efforts: list[float] = field(default_factory=list)
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


class RobotDevice(Device):
    """
    Robot device
    """

    def __init__(self, name: str):
        super().__init__(name)

    @abc.abstractmethod
    def get_state(self) -> RobotState:
        pass
    
    @abc.abstractmethod
    def set_state(self, state: RobotState):
        pass

    @abc.abstractmethod
    def start(self):
        pass

    @abc.abstractmethod
    def reboot(self):
        pass

    @abc.abstractmethod
    def stop(self):
        pass

    @abc.abstractmethod
    def reset(self):
        pass

    @abc.abstractmethod
    def apply_action(self, action: RobotAction):
        pass


class AsyncRobotDevice(RobotDevice):
    """
    Async robot device
    """

    def __init__(self, name: str):
        super().__init__(name)

    async def get_state(self) -> RobotState:
        return await AsyncUtil.run_blocking_as_async(self.get_state)

    async def set_state(self, state: RobotState):
        await AsyncUtil.run_blocking_as_async(self.set_state, state)

    async def start(self):
        await AsyncUtil.run_blocking_as_async(self.start)

    async def reboot(self):
        await AsyncUtil.run_blocking_as_async(self.reboot)

    async def stop(self):
        await AsyncUtil.run_blocking_as_async(self.stop)

    async def reset(self):
        await AsyncUtil.run_blocking_as_async(self.reset)

    async def apply_action(self, action: RobotAction):
        await AsyncUtil.run_blocking_as_async(self.apply_action, action)
