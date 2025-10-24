import asyncio
import datetime
import time
import threading
from typing import Optional

from loguru import logger
from mini_ros.utils.time_util import TimeUtil

LOG_TRACE_INFO = False


class RateLimiterAsync:
    """
    Async rate limiter. 
    Will try to reach average interval between ticks by interval_ms. (Ros-style)
    """

    def __init__(self, rate_hz: float) -> None:
        self.rate_hz: float = rate_hz
        self.prev_time: Optional[datetime.datetime] = None
        self.is_busy: bool = False
        self.sleep_time_ms: float = 0
        self.start_time: datetime.datetime = None
        self.tick_start_time: datetime.datetime = TimeUtil.now()
        self.ticks: int = 0

    def interval_ms(self) -> float:

        if not self.sleep_time_ms:
            self.sleep_time_ms = 1000.0 / self.rate_hz

        return self.sleep_time_ms

    async def wait_for_tick(self, trace_info: str = ""):
        """
        Wait for the next tick to be ready.
        """
        if LOG_TRACE_INFO:
            logger.info(f"Id: {id(self)}| RateLimiter Waiting for tick", trace_info)

        self.ticks += 1

        if self.ticks > 1:
            target_time = self.start_time + datetime.timedelta(
                milliseconds=(self.ticks - 1) * self.interval_ms()
            )

            if target_time > TimeUtil.now():
                await TimeUtil.sleep_by_ms(
                    (target_time - TimeUtil.now()).total_seconds() * 1000,
                    raise_except=True,
                )
            self.tick_start_time = TimeUtil.now()
        else:
            self.tick_start_time = TimeUtil.now()
            self.start_time = TimeUtil.now()
        
    async def reset(self):
        self.ticks = 0


class RateLimiterSync:
    """
    Sync version of RateLimiter.
    """
    def __init__(self, rate_hz: float) -> None:
        self.rate_hz: float = rate_hz
        self.prev_time: Optional[datetime.datetime] = None
        self.is_busy: bool = False
        self.sleep_time_ms: float = 0
        self.start_time: datetime.datetime = None
        self.tick_start_time: datetime.datetime = TimeUtil.now()
        self.ticks: int = 0

    def interval_ms(self) -> float:
        if not self.sleep_time_ms:
            self.sleep_time_ms = 1000.0 / self.rate_hz

        return self.sleep_time_ms

    def wait_for_tick(self, trace_info: str = "") -> None:
        """
        Wait for the next tick to be ready.
        """
        self.ticks += 1
        if self.ticks > 1:
            target_time = self.start_time + datetime.timedelta(
                milliseconds=(self.ticks - 1) * self.interval_ms()
            )
            current_time = TimeUtil.now()
            if target_time > current_time:
                time.sleep((target_time - current_time).total_seconds())
            self.tick_start_time = current_time
        else:
            self.tick_start_time = TimeUtil.now()
            self.start_time = TimeUtil.now()

    def reset(self):
        self.ticks = 0