"""
Queue, but using ZMQ.
Put using Client, get using Server.
"""
import time 
import zmq
from typing import Any
import queue
import threading
from mini_ros.common.error import NetworkError
from mini_ros.common.state import TimedData
from mini_ros.utils.time_util import TimeUtil
from loguru import logger


class NetworkQueueClient:
    """
    Client for NetworkQueue. Used for sending data to the server.
    """

    def __init__(self, name: str, port: int, timeout: int = 1000):
        self.name = name
        self.port = port
        self.timeout = timeout
        self.socket = zmq.Context().socket(zmq.DEALER)
        self.socket.connect(f"tcp://localhost:{port}")
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)  
        self._counter = 0  # Internal counter for checking data consistency

    def put(self, data: Any):
        # Send data
        timestamp = TimeUtil.now().timestamp()
        # Convert data to bytes if it's a string
        if isinstance(data, str):
            data = data.encode('utf-8')
        # Convert timestamp to bytes
        timestamp_bytes = str(timestamp).encode('utf-8')
        counter_bytes = self._counter.to_bytes(8, "big")    # 64 bits counter
        self.socket.send_multipart([timestamp_bytes, data, counter_bytes])
        logger.info(f"Client {self.name} sent data: {data.decode() if isinstance(data, bytes) else data} with counter {self._counter}")
        self._counter += 1
        # Wait for reply
        socks = dict(self.poller.poll(self.timeout))
        if socks.get(self.socket) == zmq.POLLIN:
            parts = self.socket.recv_multipart()
            recv_counter = int.from_bytes(parts[2], "big")
            logger.info(f"Recv counter: {recv_counter}, current counter: {self._counter}")
            return True
        else:
            logger.warning(f"{self.name} waiting for reply from server timed out.")
            return False

    def get(self):
        raise NetworkError(f"Only servers can get data from the queue, not clients {self.name}.")
    
    def is_empty(self):
        raise NetworkError(f"Only servers can check if the queue is empty, not clients {self.name}.")
    
    def is_full(self):
        raise NetworkError(f"Only servers can check if the queue is full, not clients {self.name}.")
    
    def size(self):
        raise NetworkError(f"Only servers can check the size of the queue, not clients {self.name}.")
    

class NetworkQueueServer:
    """
    Server for NetworkQueue. Used for receiving data from the clients.
    Put using Client, get using Server.
    """

    def __init__(self, name: str, port: int, timeout: int = 1000):
        self.queue = queue.Queue()
        self.name = name
        self.port = port
        self.timeout = timeout
        self.socket = zmq.Context().socket(zmq.ROUTER)
        self.socket.bind(f"tcp://*:{port}")
        self.socket.setsockopt(zmq.RCVTIMEO, timeout)
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)
        self._counter = 0  # Internal counter for checking data consistency

    def get(self, get_timeout: int = 10):
        # Get data from queue
        try:
            return self.queue.get(timeout=get_timeout)
        except queue.Empty:
            logger.warning(f"Queue {self.name} is empty.")
            return None

    def put(self, data: Any):
        self.queue.put(data)

    def is_empty(self):
        return self.queue.empty()
    
    def is_full(self):
        return self.queue.full()

    def size(self):
        return self.queue.qsize()
    
    def polling_loop(self, put_timeout: int = 10):
        while True:
            socks = dict(self.poller.poll(self.timeout))
            if socks.get(self.socket) == zmq.POLLIN:
                parts = self.socket.recv_multipart()
                # ROUTER socket adds client identity as first part
                # Format: [client_id, timestamp, data, counter]
                if len(parts) >= 4:
                    client_id = parts[0]
                    timestamp = float(parts[1].decode('utf-8'))
                    data = parts[2].decode('utf-8') if isinstance(parts[2], bytes) else parts[2]
                    counter = int.from_bytes(parts[3], "big")
                    timed_data = TimedData(timestamp=timestamp, data=data)
                    if self.queue.full():
                        # Drop the oldest item
                        self.queue.get()
                        self.queue.put(timed_data, timeout=put_timeout)
                    else:
                        self.queue.put(timed_data, timeout=put_timeout)
                    # Send reply to client (ROUTER needs client_id first)
                    self.socket.send_multipart([client_id, parts[1], parts[2], counter.to_bytes(8, "big")])

                    logger.info(f"Sent reply with counter {counter}")
                    self._counter = counter
            else:
                time.sleep(0.001)
    
    def start_in_thread(self):
        threading.Thread(target=self.polling_loop, daemon=True).start()
