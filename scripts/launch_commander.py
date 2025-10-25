import asyncio
import uvloop
from mini_ros.devices.io.commander_node import Console
from mini_ros.devices.io.web_wrapper import CommanderWebWrapper, bind_apis
from mini_ros.utils.config import LazyConfig
from mini_ros.utils.async_util import AsyncUtil
from mini_ros.utils.cli_util import CliUtil
from loguru import logger


async def main():
    commander_node = Console()
    await commander_node.initialize()

    commander_web_wrapper = CommanderWebWrapper(commander_node)
    commander_web_wrapper.initialize()

    is_serve_frontend = LazyConfig.get_param("misc/serve_frontend")

    if is_serve_frontend == None:
        is_serve_frontend = True

    if is_serve_frontend:
        logger.info("Serving frontend")
        bind_apis(commander_web_wrapper.app)

    await commander_web_wrapper.start()
    await commander_node.start()


if __name__ == "__main__":

    args = CliUtil.get_cli_args()

    if args["config"] is not None:
        LazyConfig.set_config_path(args["config"])

    AsyncUtil.get_run_method()(main())
