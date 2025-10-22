import argparse
import functools
import subprocess
from typing import Any, Dict


class CliUtil:

    @classmethod
    def param_map(cls) -> Dict[str, Any]:

        # TODO: Add more params as needed
        return {
            "config": {"type": str},
        }

    @classmethod
    def run_command(cls, command: str) -> subprocess.CompletedProcess:
        result = subprocess.run(command.split(), capture_output=True, text=True)
        return result

    @classmethod
    def has_sudo(cls) -> bool:
        """
        Check if sudo is available.
        Returns:
            True if sudo is available, False otherwise.
        """

        try:
            res = cls.run_command('sudo echo "SUDO is available!"')
            return res.returncode == 0
        except Exception as e:
            return False

    @classmethod
    @functools.cache
    def get_cli_args(cls) -> Dict[str, Any]:
        parser = argparse.ArgumentParser()

        for key, value in cls.param_map().items():

            if "default" in value:
                parser.add_argument(
                    f"--{key}", type=value["type"], default=value["default"]
                )
            else:
                parser.add_argument(f"--{key}", type=value["type"])

        return parser.parse_args().__dict__
