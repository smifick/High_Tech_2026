"""One command starts image collection, inventory, flight transport and local UI."""
import argparse
import asyncio
from datetime import datetime, timezone
import json
import logging
from pathlib import Path
import sys

from aiohttp import web
from websockets.asyncio.server import serve

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'drone/aruco_nav'))
from mission import load_mission
from ws_control import WsServer


class Station:
    def __init__(self, args):
        self.args = args
        self.routes = load_mission(args.routes)
        self.server = WsServer(args.ws_host, args.ws_port)
        self.process = None
        self.logfile = None
        self.flight_state = 'ожидает запуска'
        self.flight_error = None
        self.attempt = args.output / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
        self.attempt.mkdir(parents=True)

    def status(self):
        path = self.attempt / 'addresses.coverage.json'
        coverage = json.loads(path.read_text()) if path.exists() else None
        inventory = self.attempt / 'addresses.txt'
        return {'flight': self.flight_state, 'flight_error': self.flight_error,
                'connected': sorted(self.server.clients), 'expected': list(self.routes),
                'scanner': 'работает' if self.process and self.process.returncode is None else 'остановлен',
                'scanner_exit_code': self.process.returncode if self.process else None,
                'coverage': coverage, 'inventory': inventory.read_text() if inventory.exists() else '',
                'attempt': self.attempt.name}

    async def state(self, request):
        return web.json_response(self.status())

    async def action(self, request):
        # UI is loopback-only. Reject cross-site POSTs, including browser form submissions.
        if request.headers.get('X-Station-Action') != '1' or request.headers.get('Sec-Fetch-Site') == 'cross-site':
            raise web.HTTPForbidden(text='Откройте локальный интерфейс станции')
        if request.match_info['action'] == 'start':
            if self.flight_state != 'ожидает запуска':
                raise web.HTTPConflict(text='Для новой попытки перезапустите станцию')
            if not set(self.routes) <= self.server.clients.keys():
                raise web.HTTPConflict(text='Не подключены оба дрона из routes')
            if self.process is None or self.process.returncode is not None:
                raise web.HTTPConflict(text='Сбор снимков не работает: проверьте scan.log')
            self.server.stop_requested.clear()
            self.flight_state = 'маршруты выполняются'
            self.server.mission_task = asyncio.create_task(self.fly())
        elif request.match_info['action'] == 'stop':
            if self.server.mission_task and not self.server.mission_task.done():
                self.flight_state = 'ожидается завершение текущей команды и посадка'
                self.server.stop_requested.set()
        else:
            raise web.HTTPNotFound()
        return web.json_response({'ok': True})

    async def fly(self):
        try:
            await self.server.run_mission(self.routes)
            self.flight_state = 'завершено / остановлено'
        except Exception as error:
            self.flight_state = 'ошибка; проверьте аппараты'
            self.flight_error = str(error)
            logging.exception('Ошибка миссии')

    async def download(self, request):
        names = {'inventory.csv', 'addresses.txt', 'addresses.coverage.json', 'observations.jsonl'}
        name = request.match_info['name']
        if name not in names or not (self.attempt / name).is_file():
            raise web.HTTPNotFound()
        return web.FileResponse(self.attempt / name)

    async def index(self, request):
        return web.FileResponse(Path(__file__).with_name('index.html'))

    async def run(self):
        self.logfile = (self.attempt / 'scan.log').open('w')
        command = [sys.executable, '-u', str(ROOT / 'YOLO/scripts/scan_two_drones.py'),
                   '--servers', *self.args.cameras, '--headless', '--map', str(self.args.map),
                   '--match-angle', str(self.args.match_angle),
                   '--output', str(self.attempt / 'addresses.txt'), '--save-frames', str(self.attempt / 'frames')]
        app = web.Application()
        app.add_routes([web.get('/', self.index), web.get('/api/state', self.state),
                        web.post('/api/{action}', self.action), web.get('/download/{name}', self.download)])
        runner = web.AppRunner(app)
        try:
            # Bind listeners before spawning CV; failed binding leaves no child running.
            async with serve(self.server.handler, self.args.ws_host, self.args.ws_port, ping_interval=2, ping_timeout=5):
                await runner.setup()
                await web.TCPSite(runner, '127.0.0.1', self.args.port).start()
                self.process = await asyncio.create_subprocess_exec(*command, stdout=self.logfile, stderr=self.logfile)
                print(f'Интерфейс: http://127.0.0.1:{self.args.port} · попытка {self.attempt}', flush=True)
                try:
                    await asyncio.Future()
                finally:
                    # Finish landing while the WebSocket listener remains alive.
                    await self.server.stop_mission()
        finally:
            await runner.cleanup()
            if self.process and self.process.returncode is None:
                import signal
                self.process.send_signal(signal.SIGINT)
                try:
                    await asyncio.wait_for(self.process.wait(), 15)
                except TimeoutError:
                    self.process.terminate()
                    await self.process.wait()
            self.logfile.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cameras', nargs=2, required=True)
    parser.add_argument('--routes', type=Path, default=ROOT/'drone/aruco_nav/routes.example.json')
    parser.add_argument('--map', type=Path, default=ROOT/'drone/aruco_nav/aruco_map_750x450_id100.txt')
    parser.add_argument('--output', type=Path, default=ROOT/'attempts')
    parser.add_argument('--match-angle', type=int, choices=(0,90,180,270), default=90)
    parser.add_argument('--port', type=int, default=8088)
    parser.add_argument('--ws-host', default='0.0.0.0')
    parser.add_argument('--ws-port', type=int, default=8000)
    args = parser.parse_args()
    if len(set(args.cameras)) != 2:
        parser.error('Нужны два разных адреса камер')
    logging.basicConfig(level=logging.INFO)
    try:
        asyncio.run(Station(args).run())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
