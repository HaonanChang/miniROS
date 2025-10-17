from abc import ABC, abstractmethod

class Server(ABC):
    @abstractmethod
    def __init__(self, name: str):
        self.name = name

    @abstractmethod
    async def ensure_init(self):
        """
        Ensure the server is initialized.
        """
        pass
    
    @abstractmethod
    async def initialize(self):
        """
        Initialize the server.
        """
        pass