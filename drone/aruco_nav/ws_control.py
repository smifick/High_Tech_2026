"""Ground station for unchanged Sverk navigate/land WebSocket clients."""
import argparse
import asyncio
import json
import logging
import sys

from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed
from mission import load_mission, route

log = logging.getLogger('station')


class WsServer:
    def __init__(self, host='0.0.0.0', port=8000):
        self.host, self.port = host, port
        self.clients = {}
        self.pending = {}
        self.locks = {}
        self.stop_requested = asyncio.Event()
        self.mission_task = None
        self.mission_connections = set()

    async def handler(self, ws):
        robot_id = None
        try:
            robot_id = await asyncio.wait_for(ws.recv(), 5)
            if not isinstance(robot_id, str) or not robot_id.strip() or len(robot_id) > 100 or robot_id in self.clients:
                await ws.close(1008, 'Invalid or duplicate ID')
                return
            self.clients[robot_id] = ws
            self.locks[robot_id] = asyncio.Lock()
            await ws.send(json.dumps({'type': 'registered'}))
            log.info('[%s] подключён', robot_id)
            async for raw in ws:
                msg = json.loads(raw)
                if not isinstance(msg, dict):
                    raise ValueError('Ответ должен быть объектом')
                pending = self.pending.get(robot_id)
                if pending is not None:
                    command, future = pending
                    if msg.get('type') != 'result' or type(msg.get('ok')) is not bool or msg.get('command') != command:
                        raise ValueError('Неверный формат результата')
                    if not future.done():
                        future.set_result(msg)
                else:
                    raise ValueError('Неожиданный ответ без команды')
        except (ConnectionClosed, TimeoutError, ValueError, TypeError):
            log.info('[%s] соединение закрыто или ошибка протокола', robot_id)
        finally:
            if self.clients.get(robot_id) is ws:
                del self.clients[robot_id]
                self.locks.pop(robot_id, None)
                pending = self.pending.get(robot_id)
                if pending is not None and not pending[1].done():
                    pending[1].set_exception(ConnectionError(f'{robot_id}: связь потеряна'))
                if ws in self.mission_connections:
                    self.stop_requested.set()

    async def transmit(self, msg, addr, timeout=240):
        ws = self.clients.get(addr)
        if ws is None:
            raise ConnectionError(f'{addr} не подключён')
        # Legacy clients don't echo request IDs: only one outstanding command.
        async with self.locks[addr]:
            if self.clients.get(addr) is not ws:
                raise ConnectionError(f'{addr}: соединение изменилось')
            future = asyncio.get_running_loop().create_future()
            self.pending[addr] = (msg['command'], future)
            try:
                await ws.send(json.dumps(msg))
                result = await asyncio.wait_for(future, timeout)
                if not result['ok']:
                    raise RuntimeError(f'{addr}: {result.get("error", "отказ команды")}')
                return result
            except (TimeoutError, asyncio.CancelledError):
                # Never let a late reply acknowledge a subsequent command.
                await ws.close(1011, 'Command timeout or cancellation')
                raise
            finally:
                self.pending.pop(addr, None)
                if not future.done():
                    future.cancel()

    async def run_mission(self, routes):
        if len(routes) != 2:
            raise ValueError('Нужны маршруты двух дронов')
        routes = {robot: route(spec) for robot, spec in routes.items()}
        connections = {robot: self.clients.get(robot) for robot in routes}
        if any(ws is None for ws in connections.values()):
            raise ConnectionError('Для старта нужны оба дрона: ' + ', '.join(routes))
        self.mission_connections = set(connections.values())

        async def send(robot, command, params=None):
            if self.clients.get(robot) is not connections[robot]:
                raise ConnectionError(f'{robot}: исходное соединение потеряно')
            return await self.transmit({'command': command, 'params': params or {}}, robot)

        async def fly(robot, spec):
            try:
                for index, (x, y) in enumerate(spec['points']):
                    if self.stop_requested.is_set():
                        break
                    await send(robot, 'navigate', {'x': x, 'y': y, 'z': spec['height'],
                                                  'yaw': 0.0, 'frame_id': 'aruco_map'})
                    log.info('[%s] точка %d/%d достигнута', robot, index + 1, len(spec['points']))
            except Exception:
                self.stop_requested.set()
                raise
            finally:
                try:
                    await send(robot, 'land')
                    log.info('[%s] посадка подтверждена', robot)
                except Exception:
                    self.stop_requested.set()
                    log.exception('[%s] посадка не подтверждена сервером', robot)
                    raise

        group = asyncio.gather(*(fly(robot, spec) for robot, spec in routes.items()), return_exceptions=True)
        try:
            results = await asyncio.shield(group)
            for result in results:
                if isinstance(result, BaseException):
                    raise result
            log.info('Миссия остановлена' if self.stop_requested.is_set() else 'Оба маршрута завершены')
            return not self.stop_requested.is_set()
        finally:
            self.stop_requested.set()
            # Finish the current legacy command before landing; no concurrent SDK calls.
            await group
            self.mission_connections.clear()

    async def stop_mission(self):
        if self.mission_task is not None:
            self.stop_requested.set()
            await asyncio.gather(self.mission_task, return_exceptions=True)
            self.mission_task = None

    def mission_finished(self, task):
        if task.cancelled():
            log.warning('Миссия остановлена')
        elif task.exception():
            log.error('Миссия прервана: %s', task.exception())

    async def console(self):
        print('Команды: list | run ПУТЬ.json | stop | exit', flush=True)
        # Async stdin keeps stop/exit usable while both routes are running.
        reader = asyncio.StreamReader()
        protocol = asyncio.StreamReaderProtocol(reader)
        transport, _ = await asyncio.get_running_loop().connect_read_pipe(lambda: protocol, sys.stdin)
        try:
            while True:
                raw = await reader.readline()
                if not raw:
                    return
                parts = raw.decode().strip().split(maxsplit=1)
                if not parts:
                    continue
                try:
                    if parts == ['list']:
                        print(', '.join(sorted(self.clients)) or 'Нет дронов', flush=True)
                    elif parts[0] == 'run' and len(parts) == 2:
                        if self.mission_task is not None and not self.mission_task.done():
                            raise ValueError('Миссия уже выполняется')
                        routes = load_mission(parts[1])
                        if not set(routes) <= self.clients.keys():
                            raise ValueError('Сначала подключите оба дрона из routes')
                        self.stop_requested.clear()
                        self.mission_task = asyncio.create_task(self.run_mission(routes))
                        self.mission_task.add_done_callback(self.mission_finished)
                    elif parts == ['stop']:
                        await self.stop_mission()
                    elif parts == ['exit']:
                        return
                    else:
                        raise ValueError('Команды: list | run ПУТЬ.json | stop | exit')
                except (ValueError, OSError) as error:
                    print(f'Ошибка: {error}', flush=True)
        finally:
            transport.close()

    async def start(self):
        async with serve(self.handler, self.host, self.port, ping_interval=2, ping_timeout=5):
            log.info('Сервер слушает %s:%s', self.host, self.port)
            try:
                await self.console()
            finally:
                await self.stop_mission()

    def run(self):
        asyncio.run(self.start())


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8000)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    WsServer(args.host, args.port).run()
