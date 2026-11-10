import json

from websockets.asyncio.client import connect


class WsClient:
    def __init__(self, ws, robot_id):
        self.ws = ws
        self.robot_id = robot_id
        self.commands = {}

    async def handler(self, msg):
        command = self.commands[msg["command"]]
        return await command(**msg.get("params", {}))

    async def run(self):
        async with connect(self.ws) as ws:
            await ws.send(self.robot_id)

            if await ws.recv() != "ok":
                raise RuntimeError("Ошибка регистрации")

            async for raw in ws:
                try:
                    data = await self.handler(json.loads(raw))
                    response = json.dumps({"ok": True, "data": data})
                except Exception as error:
                    response = json.dumps({
                        "ok": False,
                        "error": str(error),
                    })

                await ws.send(response)