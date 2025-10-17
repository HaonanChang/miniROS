from abc import ABC, abstractmethod


class Client(ABC):
    @abstractmethod
    def __init__(self, name: str):
        self.name = name

    @abstractmethod
    async def ensure_init(self):
        """
        Ensure the client is initialized.
        """
        pass
    
    @abstractmethod
    async def initialize(self):
        """
        Initialize the client.
        """
        pass
    
    @abstractmethod
    async def recv(self):
        """
        Receive a message from the server.
        """
        pass
    
    @abstractmethod
    async def send(self, payload: object, timeout: int = 1000, max_retry: int = 3, ignore_exceptions: bool = False):
        """
        Send a message to the server.
        Args:
            payload: The payload to send.
            timeout: The timeout in milliseconds.
            max_retry: The maximum number of retries.
            ignore_exceptions: Whether to ignore exceptions.
        """
        pass
    
    @abstractmethod
    async def close(self):
        """
        Close the client.
        """
        pass
    