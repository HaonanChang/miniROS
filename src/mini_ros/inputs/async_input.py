import asyncio
import copy
from collections import defaultdict
from typing import Any, Callable, Coroutine, Dict, List, Optional, Set
from mini_ros.common.error import ReadError, WriteError


import aiomultiprocess

# from mini_ros.inputs.input_device import InputDevice, InputDeviceState
from loguru import logger
from mini_ros.utils.rate_limiter import RateLimiter
from mini_ros.utils.async_util import AsyncUtil
from mini_ros.utils.time_util import TimeUtil
from mini_ros.common.state import InputDeviceState
from mini_ros.common.driver import Driver
from mini_ros.common.device import Device

DEBUG_ASYNC_INPUT = True


class AsyncInput(Device):
    """
    AsyncIO Wrapper for InputDevice. Support multiple drivers.
    State:
        INIT -> OPEN -> READY -> RECORDING -> READY -> RECORDING -> ... -> STOPPED
    
    NOTICE: There is one thing I need to think, should I have a reading limiter?
    """

    def __init__(self, drivers: Dict[str, Driver], driver_configs: Dict[str, Any], driver_rate: Dict[str, int]):
        """
        Initialize the InputDeviceAsyncWrapper object
        """

        super().__init__(f"{'&'.join(drivers.keys())}")
        assert len(drivers) == len(driver_configs), "Number of drivers and driver configs must be the same"
        self.drivers: Dict[str, Driver] = drivers
        self.driver_configs: Dict[str, Any] = driver_configs
        self.driver_limiters: Dict[str, RateLimiter] = {driver_name: RateLimiter(driver_rate[driver_name]) for driver_name in drivers.keys()}

        # Internal state
        self.state: InputDeviceState = InputDeviceState.INIT
        self.condition: asyncio.Condition = asyncio.Condition()

        # Background task for the reading loop
        self._polling_read_tasks: Dict[str, asyncio.Task] = {}

        # Store the input value for each driver, coroutine safe
        self._input_queues: Dict[str, asyncio.Queue] = {driver_name: asyncio.Queue(maxsize=1) for driver_name in drivers.keys()}
        self._event_listeners: Dict[str, Set[Callable[..., Coroutine[Any, Any, Any]]]] = {driver_name: set([]) for driver_name in drivers.keys()}

    async def wait_for_state(self, target_state: InputDeviceState):
        async with self.condition:
            await self.condition.wait_for(lambda: self.state == target_state)

    async def set_state(self, new_state: InputDeviceState):
        async with self.condition:
            self.state = new_state
            self.condition.notify_all()

    async def initialize(self, driver_config: Any):
        """
        Perform handshake with the device and establish communication
        """

        for driver_name, driver_config in self.driver_configs.items():
            await AsyncUtil.run_blocking_as_async(self.drivers[driver_name].initialize, driver_config)
        await self.set_state(InputDeviceState.OPEN)
    
    async def start_record(self):
        """
        Start recording the input device
        """

        await self.wait_for_state(InputDeviceState.READY)
        logger.info(self.__class__.__name__, "Start record")
        await self.set_state(InputDeviceState.RECORDING)

    async def stop_record(self):
        """
        Stop recording the input device
        """

        await self.wait_for_state(InputDeviceState.RECORDING)
        logger.info(self.__class__.__name__, "Stop record")
        await self.set_state(InputDeviceState.READY)

    async def discard_data(self):
        """
        Discard the data from the input device
        """
        await self.wait_for_state(InputDeviceState.RECORDING)
        logger.info(self.__class__.__name__, "Stop record & Discard data")
        await self.set_state(InputDeviceState.READY)

    async def start(self) -> None:
        """
        Start the polling loop
        (Call this method before making any get_state() calls
        or else expect lags from the first get_state() call)
        """

        await self.wait_for_state(InputDeviceState.OPEN)
        self._polling_read_tasks = {driver_name: AsyncUtil.detach_coroutine(self._reading_loop(driver_name)) for driver_name in self.drivers.keys()}
        for driver_name in self.drivers.keys():
            self._polling_read_tasks[driver_name].add_done_callback(lambda _: self._input_queues[driver_name].task_done())
        await TimeUtil.sleep_by_ms(500)  # Wait for the reading loop to start
        await self.set_state(InputDeviceState.READY)
        
    async def add_listener(
        self, driver_name: str, listener: Callable[..., Coroutine[Any, Any, Any]]
    ) -> None:
        """
        Add a listener to the PollingInput for a specific driver.
        """

        self._event_listeners[driver_name].add(listener)

    async def get_state(self, driver_name: str):
        """
        Reading from the device. Limit reading rate by the driver rate.

        Can only be called when the input device is in RECORDING state.

        Returns:
            Any (You should know and override the type of this return value): The input value from the Polling device
        """

        await self.wait_for_state(InputDeviceState.RECORDING)

        limiter = self.driver_limiters[driver_name]
        await limiter.wait_for_tick("AsyncInput_get_state")
        # Handover wait for the value to be available to the reading loop
        val = await self._input_queues[driver_name].get()  # Block until a value is available
        await limiter.unset_busy("AsyncInput_get_state")

        return val

    async def _reading_loop(self, driver_name: str) -> None:
        """
        Polling from the driver ASAP.
        """

        limiter = self.driver_limiters[driver_name]

        while True:
            async with self.condition:
                state = self.state

            if state == InputDeviceState.STOPPED:
                break

            start_time = TimeUtil.now()
            try:
                val = await AsyncUtil.run_blocking_as_async(self.drivers[driver_name].get_state)

                # Write the input value to shared container
                # None generally means Communication Failure, ignore it
                if val is not None:
                    if self._input_queues[driver_name].full():
                        self._input_queues[driver_name].get_nowait()  # Discard the oldest value
                   
                    self._input_queues[driver_name].put_nowait(val)
                    # Broadcast the new value to all listeners
                    for listener in self._event_listeners[driver_name]:
                        AsyncUtil.detach_coroutine(listener(val))

            except asyncio.CancelledError:
                break  # Task was cancelled by some caller up in the call stack, exit
            except ReadError as e:
                logger.error(f"Error reading from {driver_name}: {e}")
                raise e

            elapsed_time = TimeUtil.get_elapsed_time_ms(start_time)
            if DEBUG_ASYNC_INPUT:
                logger.info(
                    f"{driver_name}: Polling took {elapsed_time}ms({int(float(1000) / elapsed_time)} Hz)",
                )

    async def stop(self) -> None:
        """
        Gracefully stop the reading loop.
        You should invoke this method for stopping the input device for a specific driver,
        do not override this method!
        For resource freeing, override stop() instead for a specific driver
        """
        await self.set_state(InputDeviceState.STOPPED)

        if (
            self._polling_read_tasks is not None
            and not await TimeUtil.wait_until_condition(
                condition=lambda: all(task.done() for task in self._polling_read_tasks.values()),
                timeout_ms=1000,
            )
        ):

            try:
                for task in self._polling_read_tasks.values():
                    task.cancel()
            except asyncio.CancelledError:
                pass
            except Exception as e:
                raise e

            self._polling_read_tasks = {}

        for driver_name in self.drivers.keys():
            await AsyncUtil.run_blocking_as_async(self.drivers[driver_name].stop)