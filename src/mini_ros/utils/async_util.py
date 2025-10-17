import asyncio
import functools
import pathlib
import pickle
from typing import Any, Callable, Coroutine

import aiomultiprocess
import uvloop
import threading

from asyncio import AbstractEventLoop


class AsyncUtil:
    """
    A utility class for asynchronous operations.
    """

    use_uvloop = False
    _background_loop = None
    _mutex_loop = threading.Lock()

    @classmethod
    async def run_blocking_as_async(
        cls, func: Callable, *args, **kwargs
    ) -> asyncio.Future[Any]:
        """
        Run a blocking (NON-ASYNC) function asynchronously. (Hand off to a thread pool)

        Args:
            func: The function to run asynchronously.
            *args: The arguments to pass to the function.
            **kwargs: The keyword arguments to pass to the function.

        Returns:
            The result of the function.
        """

        def wrapper():
            ret, exception = None, None

            try:
                ret = func(*args, **kwargs)
            except Exception as e:
                exception = e

            return ret, exception

        ret, exception = await asyncio.get_running_loop().run_in_executor(None, wrapper)

        if exception is not None:
            raise exception

        return ret

    @classmethod
    async def save_as_pkl(cls, obj: Any, path: pathlib.Path) -> None:
        """
        Save an object as a pickle file asynchronously. This is async-ready in terms of performance.
        """

        # Make sure the directory exists
        dir_name = path if path.is_dir() else path.parent.resolve()
        pathlib.Path(dir_name).mkdir(parents=True, exist_ok=True)

        def wrapper(data, target_path):
            with open(target_path, "wb") as f:
                pickle.dump(data, f)

        await cls.run_blocking_as_async(wrapper, obj, path)

    @classmethod
    def get_looper(cls) -> Callable:
        """
        Get the new_event_loop function for the current event loop
        """

        return uvloop.new_event_loop if cls.use_uvloop else asyncio.new_event_loop

    @classmethod
    async def run_async_with_process(cls, async_func: Callable, *args, **kwargs):
        """
        Run an async function with a dedicated process
        """

        return await aiomultiprocess.Worker(
            target=async_func,
            args=args,
            kwargs=kwargs,
            loop_initializer=cls.get_looper(),
        )

    @classmethod
    def run_async_as_blocking(cls, task: Coroutine):
        """
        Run an async function as a blocking function. Use this in an non-async context.

        Args:
            task: The async function to run.

        Returns:
            The result of the async function.
        """

        loop: AbstractEventLoop = None

        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            with cls._mutex_loop:
                if cls._background_loop is None:
                    cls._background_loop = cls.get_looper()()

                loop = cls._background_loop

        return loop.run_until_complete(task)

    @classmethod
    def get_run_method(cls):
        return uvloop.run if cls.use_uvloop else asyncio.run

    @classmethod
    def detach_coroutine(cls, coro: asyncio.Future[Any]):
        """

        Submit a coroutine to the event loop without awaiting it, effectively "detaching" from current workflow.
        The result of which can be retrieved later with the `result` method from the object returned by this function.

        Args:
            coro: The coroutine to detach.
        Returns:
            The result of the function.
        """

        return asyncio.create_task(coro)
