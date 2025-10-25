from mini_ros.network.network_queue import One2ManyRecver
from mini_ros.common.state import RobotState, RobotAction
import time

queue = One2ManyRecver(name="client", port=5555, data_type="dict")
print("Server started, waiting for clients...")
queue.start()

for i in range(1000):
    result = queue.get()
    print(result)
    time.sleep(0.1)