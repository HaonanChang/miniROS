from abc import ABC, abstractmethod
from typing import Any


class Driver(ABC):
    """
    Base class for all drivers
    """
    name: str

    @abstractmethod
    def initialize(self, driver_config: Any):
        pass

    @abstractmethod
    def get_state(self):
        pass

    @abstractmethod
    def stop(self):
        pass