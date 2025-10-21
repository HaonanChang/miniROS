from mini_ros.network.network_queue import QueueRouterRecver
import time

queue = QueueRouterRecver("queue", 5555)
print("Server started, waiting for clients...")
queue.start_in_thread()

for i in range(1000):
    result = queue.get()
    print(result)
    time.sleep(0.1)