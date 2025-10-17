from dataclasses import dataclass
from typing import Dict

import numpy as np


@dataclass
class RGBDCameraOutput:
    """
    A class to represent a output from a Intel Realsense camera.
    """

    color_frame: bytes
    depth_frame: bytes

    def __json__(self) -> Dict[str, np.ndarray]:
        return {
            "color_frame": self.color_frame,
            "depth_frame": self.depth_frame,
        }
