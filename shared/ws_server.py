import asyncio
import json

from websockets.asyncio.server import serve


class WsServer:
    def __init__(self, host="0.0.0.0", port=8000):
        self.host = host
        self.port = port
        self.clients = {}
        self.locks = {}

    async def handler(self, ws):
        robot_id = await asyncio.wait_for(ws.recv(), timeout=5)

        if not isinstance(robot_id, str) or not robot_id:
            await ws.close(1008, "Некорректный ID")
            return

        if robot_id in self.clients:
            await ws.close(1008, "ID уже подключён")
            return

        self.clients[robot_id] = ws
        self.locks[robot_id] = asyncio.Lock()

        try:
            await ws.send("ok")
            print(f"Подключён: {robot_id}")
            await ws.wait_closed()
        finally:
            del self.clients[robot_id]
            del self.locks[robot_id]
            print(f"Отключён: {robot_id}")

    async def transmit(self, msg, addr):
        ws = self.clients[addr]
        lock = self.locks[addr]

        async with lock:
            await ws.send(json.dumps(msg))
            return json.loads(await ws.recv())

    async def start(self):
        async with serve(self.handler, self.host, self.port):
            await asyncio.Future()

    def run(self):
        asyncio.run(self.start())