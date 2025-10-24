"""
Launch RDC commander for Marvin.
"""
import yaml
import os
from mini_ros.common.device import motor_config_from_json
from mini_ros.devices.motors.feetech import FeetechReader
from mini_ros.nodes.commander_node import CommanderNode
from mini_ros.devices.io.web_wrapper import CommanderWebWrapper, bind_apis
from mini_ros.utils.config import LazyConfig
from mini_ros.utils.async_util import AsyncUtil
from mini_ros.utils.cli_util import CliUtil
from mini_ros.wrapper.parallel_robot import ParallelRobotMT, ParallelRobotConfig
from mini_ros.wrapper.multi_robot import MultiRobotSystem
from loguru import logger


async def main():
    # Initialize gello
    root_dir = os.path.dirname(os.path.abspath(__file__))
    joint_config_file = os.path.join(root_dir, "../assets/gellos/joint_map_feetech.yaml")
    with open(joint_config_file, "r") as f:
        joint_config = yaml.load(f, Loader=yaml.FullLoader)
    motor_config = motor_config_from_json(joint_config)
    gello = FeetechReader(motor_config)
    gello.initialize()

    # Build 
    multi_robot = MultiRobotSystem(devices={"gello": gello})
    parallel_robot = ParallelRobotMT(multi_robot, ParallelRobotConfig(
        control_freqs={
            "gello": 60,
        },
        read_freqs={
            "gello": 60,
        },
    ))

    commander_node = CommanderNode()
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
