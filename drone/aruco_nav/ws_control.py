import asyncio
import json

from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed


class WsServer:
    def __init__(self, host="0.0.0.0", port=8000):
        self.host = host
        self.port = port
        self.clients = {}

    async def handler(self, ws):
        try:
            robot_id = await asyncio.wait_for(ws.recv(), timeout=5)
        except TimeoutError:
            await ws.close(1008, "Registration timeout")
            return
        except ConnectionClosed:
            return

        if not isinstance(robot_id, str) or not robot_id.strip():
            await ws.close(1008, "Invalid robot ID")
            return

        if robot_id in self.clients:
            await ws.close(1008, "Robot ID already connected")
            return

        self.clients[robot_id] = ws

        try:
            await ws.send(json.dumps({"type": "registered"}))
            print(f"\n[{robot_id}] подключён", flush=True)

            async for raw in ws:
                try:
                    msg = json.loads(raw)
                except (ValueError, UnicodeError):
                    print(f"\n[{robot_id}] некорректный JSON", flush=True)
                    continue

                await self.on_message(robot_id, msg)

        except ConnectionClosed:
            pass
        finally:
            del self.clients[robot_id]
            print(f"\n[{robot_id}] отключён", flush=True)

    async def on_message(self, robot_id, msg):
        print(f"\n[{robot_id}] {msg}", flush=True)

    async def transmit(self, msg, addr):
        ws = self.clients.get(addr)
        if ws is None:
            raise ConnectionError(f"{addr} не подключён")

        try:
            await ws.send(json.dumps(msg))
        except ConnectionClosed as error:
            raise ConnectionError(f"{addr} отключился") from error

    async def console(self):
        print("Команды: list | go ID X Y Z [FRAME] | land ID | exit")

        while True:
            try:
                parts = (await asyncio.to_thread(input, "> ")).split()
            except EOFError:
                return

            if not parts:
                continue

            command, *args = parts

            try:
                if command == "list":
                    print(", ".join(sorted(self.clients)) or "Нет подключённых роботов")

                elif command == "go" and len(args) in (4, 5):
                    robot_id, x, y, z, *frame = args
                    await self.transmit(
                        {
                            "command": "navigate",
                            "params": {
                                "x": float(x),
                                "y": float(y),
                                "z": float(z),
                                "frame_id": frame[0] if frame else "aruco_map",
                            },
                        },
                        addr=robot_id,
                    )
                    print(f"[{robot_id}] координаты отправлены")

                elif command == "land" and len(args) == 1:
                    await self.transmit({"command": "land"}, addr=args[0])
                    print(f"[{args[0]}] команда посадки отправлена")

                elif command == "exit":
                    return

                else:
                    print("Команды: list | go ID X Y Z [FRAME] | land ID | exit")

            except (ValueError, ConnectionError) as error:
                print(f"Ошибка: {error}")

    async def start(self):
        async with serve(self.handler, self.host, self.port):
            print(f"Сервер слушает {self.host}:{self.port}")
            await self.console()

    def run(self):
        asyncio.run(self.start())


if __name__ == "__main__":
    WsServer().run()