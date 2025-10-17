import base64
import copy
import ctypes
import gc
import inspect
import os
import pathlib
import sys
import uuid as uuid
from typing import Any, Callable, Dict, List, Union

import json5

from loguru import logger
from mini_ros.utils.async_util import AsyncUtil
from mini_ros.utils.time_util import TimeUtil


class LangUtil:
    """
    A set of utility functions to simplify boilterplates in Python.
    Mostly copied from https://github.com/LBYPatrick/naive-doc-search, which is under MIT.
    """

    @classmethod
    def get_abs_path(cls, relative_path: str) -> pathlib.Path:
        """
        Get the absolute path of a path relative to the current working directory.
        """

        return pathlib.Path(os.getcwd()) / relative_path

    @classmethod
    def str_to_bool(cls, bool_str: Union[str, bool]) -> bool:
        """
        Convert a string to a boolean.
        Args:
            bool_str: The string to convert.
        Returns:
            The boolean value of the string.
        """

        if isinstance(bool_str, bool):
            return bool_str

        yes = set(["yes", "true", "1", "on", "enabled"])
        if bool_str.lower() in yes:
            return True
        else:
            return False

    @classmethod
    async def encode_to_base64(cls, input_bytes: bytes) -> str:
        """
        Encode a bytes object to a base64 string.
        """

        def blocking_encode(ipt: bytes) -> str:
            return base64.b64encode(ipt).decode("utf-8")

        return await AsyncUtil.run_blocking_as_async(blocking_encode, input_bytes)

    @classmethod
    async def run_gc(cls):
        """
        Run garbage collection.
        """

        logger.info("Collecting garbage...")

        start_time = TimeUtil.now()

        # before_gc = sum(
        #     obj.nbytes for obj in gc.get_objects() if hasattr(obj, "nbytes")
        # )

        await AsyncUtil.run_blocking_as_async(gc.collect)

        # Linux's way of freeing memory
        if sys.platform.startswith("linux"):
            ctypes.CDLL("libc.so.6").malloc_trim(0)

        # after_gc = sum(obj.nbytes for obj in gc.get_objects() if hasattr(obj, "nbytes"))

        elapsed_time = TimeUtil.get_elapsed_time_ms(start_time)

        logger.info(f"Garbage collected! Freed {elapsed_time} ms.")

        # freed_memory_bytes = before_gc - after_gc

        # # Turn freed_memory_bytes into the closest unit
        # if freed_memory_bytes > 1024 * 1024:
        #     freed_memory_mb = freed_memory_bytes / 1024 / 1024
        #     logger.info(
        #         f"Garbage collected! Freed {freed_memory_mb:.2f} MB in {elapsed_time} ms."
        #     )
        # else:
        #     logger.info(
        #         f"Garbage collected! Freed {freed_memory_bytes} bytes in {elapsed_time} ms."
        #     )

    @classmethod
    def get_method_name(cls, obj: Union[Any, Callable]) -> str:
        """
        Get the name of a method.
        """

        return obj.__name__

    @classmethod
    def get_method_map(cls, *methods: List[Callable]) -> Dict[str, Callable]:
        """
        Get a map of method names to methods.
        """

        return {method.__name__: method for method in methods}

    @classmethod
    def find_missing_args(cls, obj: Union[Any, Dict[str, Any]], *args):
        """
        Find the missing arguments from a dict / dict-like object.
        Args:
            obj: The object to check.
            *args: The required arguments.
        Returns:
            A list of missing arguments.
        """

        victim = obj

        if not isinstance(obj, dict):
            victim = dict(obj.__dict__)

        return [arg for arg in args if arg not in victim]

    @classmethod
    def has_all_required_args(cls, obj: Union[Any, Dict[str, Any]], *args) -> bool:
        """
        Check if an object has all the required arguments.
        Args:
            obj: The object to check.
            *args: The required arguments.
        Returns:
            True if the object has all the required arguments, False otherwise.
        """

        return len(cls.find_missing_args(obj, *args)) == 0

    @classmethod
    def get_callers(cls, offset: int = 1) -> List[inspect.FrameInfo]:
        """
        Get the callers of the current function.
        """

        callers: List[inspect.FrameInfo] = copy.deepcopy(inspect.stack())

        if len(callers) >= offset:
            return callers[offset:]
        else:
            return []

    @classmethod
    def ensure_props(cls, obj: Union[Any, Dict[str, Any]], *args) -> None:
        """
        Ensure that an object has all the required arguments. Will raise an ValueError if it does not.
        Args:
            obj: The object to check.
            *args: The required arguments.
        """

        missing = cls.find_missing_args(obj, *args)

        if len(missing) > 0:
            raise ValueError(f"Missing required arguments: {missing}")

    @classmethod
    def get_dict_value(cls, mp: Dict[str, Any], key: str, default_value=None):
        """
        Read a value from a dictionary. Returns the default value if the key is not present.
        """

        return default_value if key not in mp else mp[key]

    @classmethod
    def make_uid(cls, version=4):
        """
        Make a UUID.
        """

        funcs = {1: uuid.uuid1, 3: uuid.uuid3, 4: uuid.uuid4, 5: uuid.uuid5}

        if version not in funcs.keys():
            return str(funcs[4]())
        else:
            return str(funcs[version]())

    @classmethod
    def func_hash(cls, *args, **kwargs):
        """
        Hash a function. This is used by aiocache.
        """

        mp = {}
        anon = []

        for arg in args:
            hashed = cls.get_uid()

            if arg is bool:
                anon.append("True" if arg else "False")
                continue

            try:
                hashed = str(arg)
            except Exception as e:
                print(e)
            finally:
                anon.append(hashed)

        mp = {"anon": anon, **kwargs}
        ret = json5.dumps(mp)

        return ret

    @classmethod
    def maybe(cls, val: Union[Any, None], default_val: Any):
        """
        Return the value if it is not None, otherwise return the default value.
        Equivalent to the ?? operator in other languages.
        Proposed in PEP-505 but never made it into the language.
        https://peps.python.org/pep-0505/
        Args:
            val: The value to check.
            default_val: The value to return if the left value is None.
        Returns:
            The left value if it is not None, otherwise return the default value.
        """

        return default_val if val is None else val

    @classmethod
    def get_nice_dict(cls, d: Dict[str, Any]) -> Dict[str, Any]:
        """
        Return a sorted dictionary based on key name and value type.

        Args:
            d: Input dictionary to process

        Returns:
            Dictionary containing only keys with primitive values
        """

        simple_keys = [
            k
            for k, v in d.items()
            if isinstance(v, (str, int, float, bool, type(None)))
        ]

        # Keys to recursively process
        complex_keys = [
            k for k in d.keys() if k not in simple_keys and isinstance(d[k], dict)
        ]

        other_keys = [k for k in d.keys() if k not in set(simple_keys + complex_keys)]

        return {
            **{k: d[k] for k in sorted(simple_keys)},
            **{k: d[k] for k in sorted(other_keys)},
            **{k: cls.get_nice_dict(d[k]) for k in sorted(complex_keys)},
        }
