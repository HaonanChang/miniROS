import copy
import datetime
import functools
import os
import pathlib
import random
from typing import Any, Dict, Optional, Set

import json5
import yaml

from loguru import logger
from mini_ros.utils.lang_util import LangUtil


class LazyConfig:
    """
    A global class to read and write to the config file
    Load the config file lazily, only when it is needed
    Working directory MUST be the root directory of the project
    """

    data: Dict[str, Any] = None
    config_path: pathlib.Path = None
    _is_mock: bool = None

    @classmethod
    def get_valid_extensions(cls) -> Set[str]:
        """
        Returns a list of valid config file extensions
        """
        return set(["yaml", "yml", "json", "json5", "jsonc"])

    @classmethod
    def set_config_path(cls, config_path: str, relative=True) -> pathlib.Path:
        """
        Set the path to the config file, future loads will be based on this path
        If relative is True, the config_path is relative to the current working directory

        Args:
            config_path: The path to the config file
            relative: Whether the config_path is relative to the current working directory

        Returns:
            The resolved absolutepath to the config file
        """

        target_path = None

        # Bad files
        if "." not in config_path:
            raise ValueError(
                f"Config file name must contain a file extension: {config_path}"
            )

        extension = config_path.split(".")[-1]

        if extension not in cls.get_valid_extensions():
            raise ValueError(
                f"Unsupported file extension: {extension}, must be one of: {cls.get_valid_extensions()}"
            )

        # Relative files
        if relative:
            wd = pathlib.Path(os.getcwd())
            target_path = wd.joinpath(pathlib.Path(config_path))

        else:
            target_path = pathlib.Path(config_path)

        # Check if the file exists
        if not pathlib.Path(target_path).exists():
            raise FileNotFoundError(f"Custom config file not found: {target_path}")

        # Avoid repeated loading of the same file
        if target_path != cls.config_path:
            cls.config_path = target_path
            cls.data = None

        return cls.config_path

    @classmethod
    def get_config_path(cls) -> pathlib.Path:
        """
        Returns the cls.config_path if present, or find the YAML / JSON config file in the working directory
        """

        if cls.config_path != None:
            return cls.config_path

        # Config file extensions to check for
        wd = pathlib.Path(os.getcwd())

        for ext in list(cls.get_valid_extensions()):
            target = wd.joinpath(pathlib.Path(f"./machine.{ext}"))

            if pathlib.Path(target).exists():
                logger.info(f"Detected config file: {target}")
                cls.config_path = target
                return cls.config_path

        raise FileNotFoundError("No config file found in the working directory")

    @classmethod
    def set_mock(cls, is_mock: bool):
        """
        Set the mock flag to True or False
        """

        cls._is_mock = is_mock

    @classmethod
    def is_mock(cls) -> bool:
        """
        Get the mock flag, all classes supporting mock should read from this function
        """

        if cls._is_mock is None:
            cls._is_mock = cls.get_param("misc/mock") or False

        return cls._is_mock

    @classmethod
    def get_config(cls) -> Dict[str, Any]:
        """
        Get the data from the config file
        """
        if cls.data is None:

            path = cls.get_config_path()

            raw_data = None

            with open(path, "r") as f:
                raw_data = f.read()

            if path.suffix in [".yaml", ".yml"]:
                cls.data = yaml.load(raw_data, Loader=yaml.SafeLoader)
            elif path.suffix in [".json", ".json5", ".jsonc"]:
                cls.data = json5.loads(raw_data)
        return cls.data

    @classmethod
    def get_data_folder_name(
        cls,
        time: Optional[datetime.datetime] = None,
        six_digit_id: Optional[str] = None,
    ) -> str:
        """
        Get the data folder name for the given time and six-digit ID per data-collection spec.
        """

        if time is None:
            time = datetime.datetime.now()

        if six_digit_id is None:
            six_digit_id = cls.get_session_id()

        time_str = time.strftime("%Y%m%d_%H%M%S")

        return f"{time_str}_{six_digit_id}"

    @classmethod
    def get_session_id(cls) -> str:
        """
        Get a six-digit unique session ID for the session.
        """

        # Somehow this is not unique enough but I still want six digits

        random_base = str(random.randint(1e5, 1e6 - 1))

        # I don't want leading zero, make it not so
        if random_base.startswith("0"):
            random_base[0] = str(random.randint(1, 9))

        return random_base

    @classmethod
    @functools.cache
    def get_version(cls) -> str:

        file_path = LangUtil.get_abs_path("VERSION")
        fallback_version = "UNKNOWN_VERSION"

        if not file_path.exists():
            return fallback_version

        try:
            with open(file_path, "r") as f:
                return f.read().strip()
        except FileNotFoundError:
            return fallback_version

    @classmethod
    def get_param(
        cls,
        param_path: str,
        js_style=True,
        custom_config: Optional[Dict[str, Any]] = None,
    ) -> Any:
        """
        Get a parameter from the config file.

        Args:
            param_path: The path to the parameter, for child parameters in {"a": {"b": "c"}},
            you can use "a/b" or "/a/b" to get "c"
            js_style: In case of invalid key, if js_style is True, the function will return None, otherwise it will raise an error
            custom_config: A custom config to use instead of the default one
        Returns:
            The parameter value
        """

        data = custom_config or copy.deepcopy(cls.get_config())

        nodes = [entry for entry in param_path.split("/") if len(entry) > 0]

        if len(nodes) == 0:
            return data

        for node in nodes:
            if node in data:
                data = data[node]

            # If current level is a list and the node is a digit, try to access the list at the given index
            elif type(data) == list and node.isdigit():
                idx = int(node)

                if idx < len(data):
                    data = data[idx]

            else:
                if js_style:
                    return None
                else:
                    raise ValueError(f"Invalid key: {node}")

        return data
