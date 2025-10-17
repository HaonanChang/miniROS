import asyncio
from datetime import datetime, timedelta
from typing import Any, Callable, Coroutine, Tuple
from zoneinfo import ZoneInfo


class TimeUtil:
    """
    A set of utility functions to deal with time-related operations.
    """

    @classmethod
    def now(
        cls,
        tz: ZoneInfo = None,
    ) -> datetime:
        """
        Get the current time.
        """

        return datetime.now() if tz is None else datetime.now(tz)

    @classmethod
    def get_elapsed_time(cls, start_time: datetime) -> timedelta:
        """
        Get the elapsed time as datetime.timedelta between the current time and the start time.
        """

        now = cls.now()

        if not isinstance(start_time, datetime):
            start_time = now()

        return now - start_time

    @classmethod
    def get_elapsed_time_ms(cls, start_time) -> float:
        """
        Get the elapsed time in milliseconds between the current time and the start time.
        """

        return float(cls.get_elapsed_time(start_time).total_seconds()) * 1000.0

    @classmethod
    def get_elapsed_time_ms_str(cls, start_time, digits=2) -> str:
        """
        Get the elapsed time in milliseconds as a string.
        """

        return "{0:0.{digits}f} ms".format(
            cls.get_elapsed_time_ms(start_time), digits=digits
        )

    @classmethod
    async def sleep_by_ms(cls, ms: float, raise_except=True) -> bool:
        """
        Sleep for a given number of milliseconds.

        Args:
            ms: The number of milliseconds to sleep.
            raise_except: Whether to raise an exception if the sleep is cancelled.

        Returns:
            True if the sleep was not cancelled (woken up before the sleep was complete), False otherwise.
        """

        try:
            await asyncio.sleep(ms / 1000.0)
        except asyncio.CancelledError as ce:
            # Ignore the issue but return False to the user so they know
            if raise_except:
                raise ce

            return False
        except Exception as e:
            raise e

        return True

    @classmethod
    async def bench_coroutine(
        cls, coroutine: Coroutine[Any, Any, Any]
    ) -> Tuple[float, Any]:
        """
        Benchmark the execution time of a coroutine.

        Args:
            coroutine: The coroutine to benchmark.

        Returns:
            A tuple containing the elapsed time in milliseconds and the output of the coroutine.
        """

        start_time = cls.now()
        out = await coroutine
        return cls.get_elapsed_time_ms(start_time), out

    @classmethod
    async def wait_until_condition(
        cls,
        condition: Callable[[], bool],
        timeout_ms: float = None,
    ) -> bool:
        """
        Wait for a condition to be true.

        Return:
            True if the condition was met, False if the timeout was reached.
        """

        start_time = cls.now()

        while not condition():
            await cls.sleep_by_ms(0.1, raise_except=True)

            if (
                timeout_ms is not None
                and cls.get_elapsed_time_ms(start_time) > timeout_ms
            ):
                return False

        return True

    @classmethod
    def get_elapsed_time_seconds_str(cls, start_time: datetime, digits=2) -> str:
        """
        Get the elapsed time in seconds as a string.

        Args:
            start_time: The start time to get the elapsed time from.
            digits: The number of digits to round the elapsed time to.

        Returns:
            The elapsed time in seconds as a string.
        """

        return "{0:0.{digits}f} s".format(
            cls.get_elapsed_time(start_time).total_seconds(), digits=digits
        )
