from mini_ros.network.network_queue import Many2OneSender
import time

queue = Many2OneSender("queue", 5555)

while True:
    queue.put("1, 2, 3, 4, 5")
    time.sleep(0.01)