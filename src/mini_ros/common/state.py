
"""
List all kinds of state from mini_ros.
"""

import functools
from mini_ros.utils.ordered_enum import OrderedEnum


@functools.total_ordering
class InputDeviceState(OrderedEnum):
    """
    State for recording devices.
    """
    INIT = 0
    OPEN = 1   
    READY = 2
    RECORDING = 3
    STOPPED = 4


# class InputDeviceState(OrderedEnum):
#     INIT = 0
#     READY = 1
#     ACTIVE = 2
#     STOPPED = 3