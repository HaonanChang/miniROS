from mini_ros.network.network_queue import NetworkQueueClient
import time

queue = NetworkQueueClient("queue", 5555)

while True:
    queue.put("1, 2, 3, 4, 5")
    time.sleep(0.01)