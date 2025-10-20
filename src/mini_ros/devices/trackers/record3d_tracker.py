"""
Record-3D is a tracker device.
"""

import numpy as np
from record3d import Record3DStream
import cv2
from threading import Event
from mini_ros.common.device import Tracker
from typing import Any
from mini_ros.utils.robo_util import compute_relative_pose


class Record3DTracker(Tracker):
    """
    We use Record3D as a tracker device.
    """
    name = "record3d"

    def __init__(self):
        pass

    def initialize(self, reader_config: Any = None):
        self.event = Event()
        self.session = None
        self.DEVICE_TYPE__TRUEDEPTH = 0
        self.DEVICE_TYPE__LIDAR = 1

        dev_idx = 0

        print('Searching for devices')
        devs = Record3DStream.get_connected_devices()
        print('{} device(s) found'.format(len(devs)))
        for dev in devs:
            print('\tID: {}\n\tUDID: {}\n'.format(dev.product_id, dev.udid))

        if len(devs) <= dev_idx:
            raise RuntimeError('Cannot connect to device #{}, try different index.'
                               .format(dev_idx))

        dev = devs[dev_idx]
        self.session = Record3DStream()
        self.session.on_new_frame = self._on_new_frame
        self.session.on_stream_stopped = self._on_stream_stopped
        self.session.connect(dev)  # Initiate connection and start capturing
        self.anchor_pose = np.array([0, 0, 0, 1, 0, 0, 0])

    def reanchor(self):
        self.anchor_pose = self.get_state()

    def _on_new_frame(self):
        """
        This method is called from non-main thread, therefore cannot be used for presenting UI.
        """
        self.event.set()  # Notify the main thread to stop waiting and process new frame.

    def _on_stream_stopped(self):
        print('Stream stopped')

    def stop(self):
        pass

    def get_state(self):
        self.event.wait()  # Wait for new frame to arrive

        camera_pose = self.session.get_camera_pose()  # Quaternion + world position (accessible via camera_pose.[x,y,z,qw,qx,qy,qz])

        self.event.clear()
        cur_pose = np.array([camera_pose.tx, camera_pose.ty, camera_pose.tz, camera_pose.qw, camera_pose.qx, camera_pose.qy, camera_pose.qz])
        # Relative to anchor pose
        rel_pose = compute_relative_pose(self.anchor_pose, cur_pose)
        return rel_pose