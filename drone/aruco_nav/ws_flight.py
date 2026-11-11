import argparse
import asyncio

import sverk_interfaces
import json

from websockets.asyncio.client import connect

class WsClient:
    def __init__(self, ws, robot_id):
        self.uri = ws          # Адрес ноутбука, например ws://10.11.3.208:8000
        self.robot_id = robot_id
        self.ws = None         # Открытое WebSocket-соединение

    async def transmit(self, msg):
        if self.ws is None:
            raise ConnectionError("Нет соединения с сервером")

        await self.ws.send(json.dumps(msg))

    async def run(self):
        async with connect(self.uri) as ws:
            await ws.send(self.robot_id)

            raw = await asyncio.wait_for(ws.recv(), timeout=5)
            try:
                reply = json.loads(raw)
            except (ValueError, UnicodeError):
                raise RuntimeError(f"Ошибка регистрации, ответ сервера: {raw!r}") from None

            if not isinstance(reply, dict) or reply.get("type") != "registered":
                raise RuntimeError(f"Ошибка регистрации, ответ сервера: {raw!r}")

            self.ws = ws
            print(f"[{self.robot_id}] зарегистрирован на {self.uri}", flush=True)

            try:
                async for raw in ws:
                    await self.handler(json.loads(raw)
            finally:
                self.ws = None


class DroneClient(WsClient):
    def __init__(self, ws, robot_id, drone):
        super().__init__(ws, robot_id)
        self.control = drone.control
        self.in_flight = False
        self.takeoff_attempted = False



    def navigate(self, x, y, z=1.7, yaw=0.0, frame_id="aruco_map"):
        if not self.in_flight:
            self.takeoff_attempted = True

            self.control.navigate_wait(
                x=0.0,
                y=0.0,
                z=1.7,
                frame_id="map",
                auto_arm=True,
                speed=0.3,
                tolerance=0.15,
                timeout=30.0,
            )

            telemetry = self.control.get_telemetry(frame_id="map")
            if not telemetry.connected or not telemetry.armed:
                raise RuntimeError("Нет связи с контроллером или моторы не включены")

            self.in_flight = True

        self.control.navigate_wait(
            x=float(x),
            y=float(y),
            z=float(z),
            yaw=float(yaw),
            frame_id=frame_id,
            speed=0.3,
            tolerance=0.15,
            timeout=60.0,
        )

    def land(self):
        if self.takeoff_attempted:
            self.control.land()
            self.in_flight = False
            self.takeoff_attempted = False

    async def handler(self, msg):
        command = msg.get("command")

        try:
            if command == "navigate":
                await asyncio.to_thread(self.navigate, **msg.get("params", {}))
            elif command == "land":
                await asyncio.to_thread(self.land)
            else:
                raise ValueError(f"Неизвестная команда: {command}")

            response = {"type": "result", "command": command, "ok": True}

        except Exception as error:
            response = {
                "type": "result",
                "command": command,
                "ok": False,
                "error": str(error),
            }

        await self.transmit(response)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--server", required=True)
    parser.add_argument("--id", required=True)
    args = parser.parse_args()

    drone = sverk_interfaces.init(Nodename=f"fly_scan_{args.id}")
    client = DroneClient(args.server, args.id, drone)

    try:
        asyncio.run(client.run())
    finally:
        try:
            client.land()
        finally:
            drone.close()


if __name__ == "__main__":
    main()

# python3 drone_client.py --server ws://10.11.3.208:8000 --id drone1