"""
Shared error classes
"""

class ReadError(Exception):
    """
    Error when reading from a device
    """
    pass

class WriteError(Exception):
    """
    Error when writing to a device
    """

class RobotError(Exception):
    """Base class for all robot-related errors."""

    pass


class RobotExecuteError(RobotError):
    """Robot Execute Error
    Code:
     - "OUT_OF_RANGE"
     - "EXTREME_FAST_MOTION"
     - "TIMEOUT"
     - "EXECUTION_FAILURE"
    """

    def __init__(self, message, code=None):
        super().__init__(message)
        self.code = code or "EXECUTION_FAILURE"
        # self.joint_id = joint_id

class NetworkError(Exception):
    """
    Error when communicating over the network
    """
    pass