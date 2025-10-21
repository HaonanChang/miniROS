"""
Queue, but using ZMQ.
Put using Client, get using Server.
"""
import time 
import zmq
import numpy as np
from typing import Any
import queue
import threading
from mini_ros.common.error import NetworkError
from mini_ros.common.state import TimedData
from mini_ros.utils.time_util import TimeUtil
from loguru import logger
from mini_ros.utils.net_util import NetUtil


class NetworkQueueClient:
    """
    Client for NetworkQueue. Used for sending data to the server.
    """

    def __init__(self, name: str, port: int, timeout: int = 1000, data_type: str = 'auto'):
        self.name = name
        self.port = port
        self.timeout = timeout
        self.data_type = data_type
        self.socket = zmq.Context().socket(zmq.DEALER)
        self.socket.connect(f"tcp://localhost:{port}")
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)  
        self._counter = 0  # Internal counter for checking data consistency
        self._max_counter = 2**63 - 1  # Maximum safe counter value (signed 64-bit)

    def put(self, data: Any, code: str = "normal"):
        """
        Put data into the queue.
        Send the data to the server and wait for reply through ZMQ.
        Send format: [timestamp, code, data, counter]
        Recv format: [timestamp, code, counter]
        Return: True if successful, False if timed out.
        """
        # Send data
        timestamp = TimeUtil.now().timestamp()
        # Convert data to bytes if it's a string
        encoded_data = NetUtil.encode(data, self.data_type)
        # Convert timestamp to bytes
        timestamp_bytes = str(timestamp).encode('utf-8')
        counter_bytes = self._counter.to_bytes(8, "big")    # 64 bits counter
        self.socket.send_multipart([timestamp_bytes, code.encode('utf-8'), encoded_data, counter_bytes])
        logger.info(f"Client {self.name} sent data: {data.decode() if isinstance(data, bytes) else data} with counter {self._counter}")
        # Prevent counter overflow
        if self._counter >= self._max_counter:
            logger.warning(f"Counter overflow detected, resetting to 0")
            self._counter = 0
        else:
            self._counter += 1
        # Wait for reply
        socks = dict(self.poller.poll(self.timeout))
        if socks.get(self.socket) == zmq.POLLIN:
            parts = self.socket.recv_multipart()
            # [timestamp, code, counter]
            recv_counter = int.from_bytes(parts[2], "big")
            recv_code = parts[1].decode('utf-8')
            logger.info(f"Recv code: {recv_code}, counter: {recv_counter}, current counter: {self._counter}")
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

    def __init__(self, name: str, port: int, timeout: int = 1000, data_type: str = 'auto'):
        self.queue = queue.Queue()
        self.name = name
        self.port = port
        self.timeout = timeout
        self.data_type = data_type
        self.socket = zmq.Context().socket(zmq.ROUTER)
        self.socket.bind(f"tcp://*:{port}")
        self.socket.setsockopt(zmq.RCVTIMEO, timeout)
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)
        self._counter = 0  # Internal counter for checking data consistency
        self._max_counter = 2**63 - 1  # Maximum safe counter value (signed 64-bit)

    def get(self, timeout: int = 10):
        # Get data from queue
        try:
            return self.queue.get(timeout=timeout)
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
    
    def polling_loop(self, timeout: int = 10):
        while True:
            try:
                socks = dict(self.poller.poll(self.timeout))
                if socks.get(self.socket) == zmq.POLLIN:
                    parts = self.socket.recv_multipart()
                    # ROUTER socket adds client identity as first part
                    # Format: [client_id, timestamp, code, encoded_data, counter]
                    if len(parts) >= 5:
                        client_id = parts[0]
                        timestamp = float(parts[1].decode('utf-8'))
                        code = parts[2].decode('utf-8')
                        encoded_data = parts[3]
                        counter = int.from_bytes(parts[4], "big")
                        
                        # Decode the data
                        try:
                            decoded_data = NetUtil.decode(encoded_data, self.data_type)
                        except Exception as e:
                            logger.error(f"Failed to decode data: {e}")
                            # Send error reply
                            self.socket.send_multipart([client_id, parts[1], "error".encode('utf-8'), counter.to_bytes(8, "big")])
                            continue
                        
                        timed_data = TimedData(timestamp=timestamp, code=code, data=decoded_data)
                        if self.queue.full():
                            # Drop the oldest item
                            self.queue.get()
                            self.queue.put(timed_data, timeout=timeout)
                        else:
                            self.queue.put(timed_data, timeout=timeout)
                        # Send reply to client (ROUTER needs client_id first)
                        self.socket.send_multipart([client_id, parts[1], parts[2], counter.to_bytes(8, "big")])

                        logger.info(f"Sent reply with counter {counter}")
                        self._counter = counter
                    else:
                        logger.warning(f"Received message with insufficient parts: {len(parts)}")
                else:
                    time.sleep(0.001)
            except Exception as e:
                logger.error(f"Error in polling loop: {e}")
                time.sleep(0.1)  # Brief pause before retrying
    
    def start_in_thread(self):
        threading.Thread(target=self.polling_loop, daemon=True).start()
