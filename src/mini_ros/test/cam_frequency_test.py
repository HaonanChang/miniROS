import threading
from mini_ros.common.device import Camera

from loguru import logger


class CameraFrequencyTest:
    """
    Camera frequency test.
    Read & Write at the same time.
    """
    def __init__(self, camera: Camera, write_freq: int, read_freq: int):
        self.camera = camera
        self.write_freq = write_freq
        self.read_freq = read_freq
        self.write_thread = None
        self.read_thread = None
        self.write_log = []
        self.read_log = []

    def start(self):
        pass

    def join(self):
        pass

    def write_loop(self):
        pass

    def read_loop(self):
        pass


class MultiThreadCameraFrequencyTest(CameraFrequencyTest):
    """
    Multi-thread camera frequency test.
    """
    def __init__(self, camera: Camera, write_freq: int, read_freq: int):
        super().__init__(camera, write_freq, read_freq)
        self.write_thread = threading.Thread(target=self.write_loop)
        self.read_thread = threading.Thread(target=self.read_loop)

    def start(self):
        self.write_thread.start()
        self.read_thread.start()


################################## Frame rate analyzing ##################################
