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