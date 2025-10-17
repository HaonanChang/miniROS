import abc
from abc import abstractmethod
from typing import Any

class Device(abc.ABC):
    """
    Device base class
    """
    
    def __init__(self, name: str):
        self.name = name