"""
Multiple input devices.
"""
import asyncio
import copy
from collections import defaultdict
from typing import Any, Callable, Coroutine, Dict, List, Optional, Set
from mini_ros.common.error import ReadError, WriteError


import aiomultiprocess

# from mini_ros.inputs.input_device import InputDevice, InputDeviceState
from loguru import logger
from mini_ros.utils.rate_limiter import RateLimiterAsync
from mini_ros.utils.async_util import AsyncUtil
from mini_ros.utils.time_util import TimeUtil
from mini_ros.common.state import InputDeviceState, TimedData
from mini_ros.common.device import Device

DEBUG_ASYNC_INPUT = True


class MultiInput(Device):
    """
    Multiple input devices.
    """
    def __init__(self, devices: Dict[str, Device]):
        super().__init__(f"{'&'.join(devices.keys())}")
        self.devices = devices

    def initialize(self):
        for device in self.devices.values():
            device.initialize()

    def start(self):
        for device in self.devices.values():
            device.start()

    def stop(self):
        for device in self.devices.values():
            device.stop()

    def pause(self):
        for device in self.devices.values():
            device.pause()

    def start_record(self, episode_name: str):
        for device in self.devices.values():
            device.start_record(episode_name)

    def stop_record(self):
        for device in self.devices.values():
            device.stop_record()

    def get_state(self):
        raise NotImplementedError("get_state is not supported for multi-input")

    def get_state_from(self, device_name: str):
        if device_name not in self.devices.keys():
            raise ValueError(f"Device {device_name} not found in multi-input")
        return self.devices[device_name].get_state()

    def is_active(self) -> bool:
        active_flag = True
        for device in self.devices.values():
            if not device.is_active():
                active_flag = False
                logger.warning(f"Device {device.name} is not active")
                break
        return active_flag
    
    def is_alive(self) -> bool:
        alive_flag = True
        for device in self.devices.values():
            if not device.is_alive():
                alive_flag = False
                logger.warning(f"Device {device.name} is not alive")
                break
        return alive_flag

    def device_keys(self) -> List[str]:
        return list(self.devices.keys())

    def device(self, device_name: str) -> Device:
        if device_name not in self.devices.keys():
            raise ValueError(f"Device {device_name} not found in multi-input")
        return self.devices[device_name]