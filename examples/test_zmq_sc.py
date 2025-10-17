"""
Test the ZMQ server and client.
"""

import asyncio

from mini_ros.impl.zmq_server import ZMQServer
from mini_ros.impl.zmq_client import ZMQClient


async def test_zmq_server_client():
    server = ZMQServer("server", "127.0.0.1", 5555)
    client = ZMQClient("client", "127.0.0.1", 5555)

    await server.initialize()
    await client.initialize()

    await client.send("Hello, world!")
    response = await server.recv()
    print(response)


if __name__ == "__main__":
    asyncio.run(test_zmq_server_client())