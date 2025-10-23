import subprocess
from typing import Any, Coroutine

from loguru import logger
from mini_ros.utils.async_util import AsyncUtil


class SystemUtil:
    @classmethod
    async def run_command_async(
        cls,
        command: str,
    ) -> Coroutine[Any, Any, subprocess.CompletedProcess]:
        """
        Run a command and return the completed process.
        """

        return await AsyncUtil.run_blocking_as_async(subprocess.run, command)

    @classmethod
    def run_command(cls, command: str, *args, **kwargs) -> subprocess.CompletedProcess:
        """
        Run a command and return the completed process.

        Args:
            command (str): The command to run.

        Returns:
            subprocess.CompletedProcess: The completed process.
        """

        tokens = command.split(" ")
        proc = subprocess.run(tokens, *args, **kwargs)

        logger.info(f"Ran command: {command}, status code: {proc.returncode}")

        return proc
