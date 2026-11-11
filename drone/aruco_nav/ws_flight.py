import argparse
import asyncio
import json
import logging
import math
import time

import sverk_interfaces
from websockets.asyncio.client import connect

log = logging.getLogger("drone_ws")


def check_result(result, operation):
    if result is False or getattr(result, "success", None) is False:
        detail = getattr(result, "message", "") or "SDK вернул отказ"
        raise RuntimeError(f"{operation}: {detail}")


class DroneClient:
    def __init__(self, uri, robot_id, drone):
        self.uri = uri
        self.robot_id = robot_id
        self.control = drone.control
        self.in_flight = False
        self.takeoff_attempted = False

    def navigate(self, x, y, z=1.7, yaw=0.0, frame_id="aruco_map"):
        x, y, z, yaw = map(float, (x, y, z, yaw))
        if not all(math.isfinite(value) for value in (x, y, z, yaw)):
            raise ValueError("Координаты и yaw должны быть конечными числами")

        if not self.in_flight:
            # Моторы могут включиться, даже если вызов взлёта вернёт ошибку.
            self.takeoff_attempted = True
            result = self.control.navigate_wait(
                x=0.0,
                y=0.0,
                z=1.7,
                frame_id="map",
                auto_arm=True,
                speed=0.3,
                tolerance=0.15,
                timeout=30.0,
            )
            check_result(result, "Взлёт")

            telemetry = self.control.get_telemetry(frame_id="map")
            if not telemetry.connected or not telemetry.armed:
                raise RuntimeError("Нет связи с контроллером или моторы не включены")

            self.in_flight = True

        result = self.control.navigate_wait(
            x=x,
            y=y,
            z=z,
            yaw=yaw,
            frame_id=frame_id,
            speed=0.3,
            tolerance=0.15,
            timeout=60.0,
        )
        check_result(result, "Навигация")

    def land(self):
        if not self.takeoff_attempted:
            return

        last_error = "нет подтверждения выключения моторов"

        for attempt in range(1, 4):
            log.info("Посадка, попытка %d/3", attempt)

            try:
                check_result(self.control.land(), "Посадка")
            except Exception as error:
                last_error = str(error)
                log.warning("Ошибка команды посадки: %s", error)

            # Команда могла дойти до контроллера, даже если ответ потерялся.
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                try:
                    telemetry = self.control.get_telemetry(frame_id="map")
                    if telemetry.connected and not telemetry.armed:
                        self.in_flight = False
                        self.takeoff_attempted = False
                        log.info("Моторы выключены")
                        return

                    if telemetry.connected:
                        last_error = "моторы ещё включены"
                    else:
                        last_error = "нет связи с полётным контроллером"

                except Exception as error:
                    last_error = str(error)

                time.sleep(0.25)

        raise RuntimeError(f"Посадка не подтверждена: {last_error}")

    async def handler(self, ws, raw):
        command = None

        try:
            msg = json.loads(raw)
            command = msg["command"]

            if command == "navigate":
                await asyncio.to_thread(self.navigate, **msg.get("params", {}))
            elif command == "land":
                await asyncio.to_thread(self.land)
            else:
                raise ValueError(f"Неизвестная команда: {command}")

        except Exception as error:
            log.exception("Ошибка команды %r", command)

            if command != "land":
                # Если взлёт ещё не начинался, land() сразу вернётся.
                await asyncio.to_thread(self.land)
            else:
                # Прекращаем принимать команды. main() повторит посадку.
                raise

            await ws.send(json.dumps({
                "type": "result",
                "command": command,
                "ok": False,
                "error": str(error),
            }))
            return

        await ws.send(json.dumps({
            "type": "result",
            "command": command,
            "ok": True,
        }))

    async def receive(self, ws):
        async for raw in ws:
            await self.handler(ws, raw)

    async def run(self):
        async with connect(self.uri, ping_interval=5, ping_timeout=10) as ws:
            await ws.send(self.robot_id)

            raw = await asyncio.wait_for(ws.recv(), timeout=5)
            try:
                reply = json.loads(raw)
            except ValueError:
                raise RuntimeError(f"Ошибка регистрации: {raw!r}") from None

            if not isinstance(reply, dict) or reply.get("type") != "registered":
                raise RuntimeError(f"Ошибка регистрации: {raw!r}")

            log.info("[%s] зарегистрирован на %s", self.robot_id, self.uri)

            receive_task = asyncio.create_task(self.receive(ws))
            closed_task = asyncio.create_task(ws.wait_closed())

            try:
                done, _ = await asyncio.wait(
                    (receive_task, closed_task),
                    return_when=asyncio.FIRST_COMPLETED,
                )
                if receive_task in done:
                    await receive_task
                else:
                    log.warning("Соединение с сервером закрыто")
            finally:
                receive_task.cancel()
                closed_task.cancel()
                await asyncio.gather(
                    receive_task, closed_task, return_exceptions=True
                )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--server", required=True)
    parser.add_argument("--id", required=True)
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")

    drone = sverk_interfaces.init(Nodename=f"fly_scan_{args.id}")
    client = DroneClient(args.server, args.id, drone)

    try:
        asyncio.run(client.run())
    except KeyboardInterrupt:
        log.info("Остановка клиента")
    finally:
        try:
            client.land()
        except Exception:
            log.critical("Посадка не подтверждена при завершении клиента", exc_info=True)
            raise
        finally:
            drone.close()


if __name__ == "__main__":
    main()

# python3 drone_client.py --server ws://10.11.3.208:8000 --id drone1