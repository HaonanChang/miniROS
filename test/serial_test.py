from mini_ros.devices.robots.pika_gripper import PikaGripper, PikaGripperConfig
import time
from mini_ros.devices.io.recorder import EpisodeRecorder, RecorderConfig

recorder = EpisodeRecorder(RecorderConfig(name="pika-1", fps=60, max_episode_length=200, data_root_dir="./tmp_data"))
robot = PikaGripper(PikaGripperConfig(port="/dev/ttyUSB1", read_size=0))
robot.bind_recorder(recorder)
robot.initialize()
robot.start()
robot.start_record(episode_name="e_000000")
for i in range(10):
    state = robot.get_state(is_record=True)
    time.sleep(0.01)
print("Stop recording")
robot.stop_record()
robot.stop()