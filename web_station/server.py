"""Local browser station. Never sends commands or opens cameras on page load."""
import argparse
import asyncio
from collections import deque
import hmac
import json
import os
from pathlib import Path
import secrets
import sys
import time
from urllib.parse import urlsplit

from aiohttp import web, ClientSession, ClientTimeout
from websockets.asyncio.server import serve

ROOT = Path(os.environ.get('HITECH_PROJECT_ROOT', Path(__file__).resolve().parents[1]))
sys.path[:0] = [str(ROOT/'drone/aruco_nav'), str(ROOT/'YOLO/scripts')]
from ws_control import WsServer
from mission import load_mission, validate_routes, number
from snapshot_camera import snapshot_url
from rover_adapter import RoverAdapter

STATIC = Path(__file__).with_name('static')
DEFAULT_CONFIG = Path(__file__).with_name('config.json')


class Station:
    def __init__(self, config, output=None):
        self.config = config
        self.output = Path(output or ROOT/'runs/web_station/addresses.txt')
        self.token = secrets.token_urlsafe(32)
        self.events = deque(maxlen=100)
        self.jobs = {}
        self.manual = {}
        self.land_requested = set()
        self.camera_tasks = {}
        self.camera_stop = asyncio.Event()
        self.cv_lock = asyncio.Lock()
        self.rover = RoverAdapter(config['rover']['base_url'], self.log) if config.get('rover') else None
        self.rover_camera_task = None
        self.rover_camera_stop = asyncio.Event()
        self.rover_camera_mode = 'off'
        self.rover_camera_info = {'frame':0, 'received_at':None, 'error':None, 'detections':[]}
        self.processor = None
        self.inventory = None
        self.camera_mode = 'off'
        self.camera_info = {key: {'frame': 0, 'received_at': None, 'error': None, 'detections': []}
                            for key in config['drones']}
        self.images = {}
        self.progress = {}
        self.session = None
        self.map_path = ROOT/config['map']
        self.map = []
        for line in self.map_path.read_text().splitlines():
            if line.strip() and not line.lstrip().startswith('#'):
                fields = line.split()
                self.map.append({'id': int(fields[0]), 'x': float(fields[2]), 'y': float(fields[3])})
        self.routes = load_mission(ROOT/config['routes'])
        if set(self.routes) != set(config['drones']):
            raise ValueError('ID в маршрутах и config.json должны совпадать')
        self.robot = ObservedServer(self)
        self.log('Станция готова. Камеры выключены; команды не отправлены.')

    def log(self, text, level='info'):
        self.events.appendleft({'time': time.time(), 'text': str(text), 'level': level})

    def mission_active(self):
        return self.robot.mission_task is not None and not self.robot.mission_task.done()

    def launch(self, name, coroutine):
        job_id = secrets.token_hex(6)
        self.jobs[job_id] = {'id': job_id, 'name': name, 'status': 'running', 'started': time.time()}
        self.log(name)
        async def work():
            try:
                await coroutine
                self.jobs[job_id]['status'] = 'done'
                self.log(name + ': завершено')
            except Exception as error:
                self.jobs[job_id].update(status='error', error=str(error))
                self.log(f'{name}: {error}', 'error')
            finally:
                self.jobs[job_id]['finished'] = time.time()
        task = asyncio.create_task(work())
        # Bound state while retaining all active jobs.
        for key in list(self.jobs)[:-50]:
            if self.jobs[key]['status'] != 'running':
                del self.jobs[key]
        return job_id, task

    def command(self, robot_id, command, params):
        if robot_id not in self.config['drones']:
            raise ValueError('Неизвестный дрон')
        if robot_id not in self.robot.clients:
            raise ValueError('Дрон не подключён к серверу команд')
        if command not in ('navigate', 'land'):
            raise ValueError('Поддерживаются navigate и land')
        if command == 'land' and self.mission_active():
            self.robot.stop_requested.set()
            self.log('Остановка общей миссии: посадка обоих после текущих команд', 'warning')
            return {'queued': True}
        if self.mission_active():
            raise ValueError('Сначала остановите общую миссию')
        if robot_id in self.manual:
            if command == 'land':
                self.land_requested.add(robot_id)
                self.log(f'{robot_id}: посадка после текущей команды', 'warning')
                return {'queued': True}
            raise ValueError('Дрон уже выполняет команду')
        if command == 'navigate':
            params = {key: number(params.get(key), key, low, high) for key, low, high in (
                ('x', -20, 20), ('y', -20, 20), ('z', .2, 5))}
            params.update(yaw=0.0, frame_id='aruco_map')
        else:
            params = {}
        connection = self.robot.clients[robot_id]
        async def action():
            try:
                await self.robot.transmit({'command': command, 'params': params}, robot_id)
            finally:
                try:
                    if robot_id in self.land_requested and command != 'land':
                        if self.robot.clients.get(robot_id) is not connection:
                            raise ConnectionError('Связь потеряна: посадка не подтверждена')
                        await self.robot.transmit({'command': 'land'}, robot_id)
                finally:
                    self.land_requested.discard(robot_id)
                    self.manual.pop(robot_id, None)
        job_id, task = self.launch(f'{robot_id}: {command}', action())
        self.manual[robot_id] = task
        return {'job': job_id}

    def start_mission(self, routes):
        if self.mission_active() or self.manual:
            raise ValueError('Дождитесь завершения текущих команд')
        checked = validate_routes(routes)
        if set(checked) != set(self.config['drones']):
            raise ValueError('Маршруты должны соответствовать двум ID из настроек')
        if not set(checked) <= self.robot.clients.keys():
            raise ValueError('Сначала подключите оба дрона к серверу команд')
        self.routes = checked
        self.progress.clear()
        self.robot.stop_requested.clear()
        job, task = self.launch('Общий маршрут', self.robot.run_mission(checked))
        self.robot.mission_task = task
        return {'job': job}

    def stop_all(self):
        if self.mission_active():
            self.robot.stop_requested.set()
            self.log('Запрошена остановка маршрутов и посадка обоих', 'warning')
        else:
            for robot_id in self.config['drones']:
                if robot_id in self.robot.clients:
                    self.command(robot_id, 'land', {})
        return {'queued': True}

    async def start_cameras(self, mode):
        if mode not in ('raw', 'cv'):
            raise ValueError('Режим камеры: raw или cv')
        if self.camera_tasks:
            raise ValueError('Сначала остановите камеры')
        self.camera_mode = mode
        self.camera_stop.clear()
        self.camera_tasks = {key: asyncio.create_task(self.camera_loop(key)) for key in self.config['drones']}
        self.log('Камеры включены: ' + mode)

    async def stop_cameras(self):
        self.camera_stop.set()
        await asyncio.gather(*self.camera_tasks.values(), return_exceptions=True)
        self.camera_tasks.clear()
        self.camera_mode = 'off'
        self.log('Камеры остановлены')

    async def camera_loop(self, robot_id):
        info = self.camera_info[robot_id]
        url = snapshot_url(self.config['drones'][robot_id]['camera'])
        while not self.camera_stop.is_set():
            started = time.monotonic()
            try:
                async with self.session.get(url, timeout=ClientTimeout(total=6)) as response:
                    response.raise_for_status()
                    if 'image/jpeg' not in response.headers.get('Content-Type', '').lower():
                        raise ValueError('Камера вернула не JPEG')
                    data = bytearray()
                    async for chunk in response.content.iter_chunked(65536):
                        data.extend(chunk)
                        if len(data) > 16*1024*1024:
                            raise ValueError('Слишком большой снимок')
                received = time.time()
                if self.camera_stop.is_set():
                    break
                import cv2
                import numpy as np
                frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    raise ValueError('Некорректный JPEG')
                result = {'detections': [], 'markers': {}}
                annotated = bytes(data)
                if self.camera_mode == 'cv':
                    async with self.cv_lock:
                        if self.camera_stop.is_set():
                            break
                        if self.processor is None:
                            from box_detector import BoxProcessor
                            from scan_two_drones import Inventory
                            self.processor = await asyncio.to_thread(BoxProcessor, ROOT/self.config['weights'],
                                self.config.get('device', 'cpu'), .25, self.config['dictionary'])
                            self.inventory = Inventory(self.map_path)
                        output, result = await asyncio.to_thread(self.processor.process, frame)
                        ok, jpeg = cv2.imencode('.jpg', output)
                        if not ok:
                            raise RuntimeError('Ошибка кодирования результата')
                        annotated = jpeg.tobytes()
                        self.inventory.update(robot_id, result)
                        self.inventory.save(self.output)
                self.images[robot_id] = {'raw': bytes(data), 'cv': annotated}
                info.update(frame=info['frame']+1, received_at=received, error=None,
                            detections=result['detections'], markers=result.get('markers', {}),
                            cycle_ms=round((time.monotonic()-started)*1000))
            except Exception as error:
                message = str(error) or type(error).__name__
                if info['error'] != message:
                    self.log(f'{robot_id}: камера — {message}', 'error')
                info['error'] = message
            try:
                await asyncio.wait_for(self.camera_stop.wait(), max(.05, 1-(time.monotonic()-started)))
            except TimeoutError:
                pass

    async def start_rover_camera(self, mode):
        if not self.rover:
            raise ValueError('Ровер не настроен')
        if mode not in ('raw','cv'):
            raise ValueError('Режим камеры: raw или cv')
        if self.rover_camera_task:
            raise ValueError('Сначала остановите камеру ровера')
        self.rover_camera_mode=mode
        self.rover_camera_stop.clear()
        self.rover_camera_task=asyncio.create_task(self.rover_camera_loop())
        self.log('Камера ровера включена: '+mode)

    async def stop_rover_camera(self):
        self.rover_camera_stop.set()
        if self.rover_camera_task:
            await self.rover_camera_task
        self.rover_camera_task=None
        self.rover_camera_mode='off'

    async def rover_camera_loop(self):
        info=self.rover_camera_info
        url=self.config['rover']['camera_url']
        while not self.rover_camera_stop.is_set():
            started=time.monotonic()
            try:
                async with self.session.get(url, timeout=ClientTimeout(total=5)) as response:
                    response.raise_for_status()
                    if 'image/jpeg' not in response.headers.get('Content-Type','').lower():
                        raise ValueError('Камера ровера вернула не JPEG')
                    data=bytearray()
                    async for chunk in response.content.iter_chunked(65536):
                        data.extend(chunk)
                        if len(data)>16*1024*1024:raise ValueError('Слишком большой снимок')
                received=time.time()
                import cv2
                import numpy as np
                frame=cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_COLOR)
                if frame is None:raise ValueError('Некорректный кадр ровера')
                result={'detections':[]}
                annotated=bytes(data)
                if self.rover_camera_mode=='cv':
                    async with self.cv_lock:
                        if self.rover_camera_stop.is_set():break
                        if self.processor is None:
                            from box_detector import BoxProcessor
                            from scan_two_drones import Inventory
                            self.processor=await asyncio.to_thread(BoxProcessor, ROOT/self.config['weights'],
                                self.config.get('device','cpu'),.25,self.config['dictionary'])
                            self.inventory=Inventory(self.map_path)
                        output,result=await asyncio.to_thread(self.processor.process,frame,associate_markers=False)
                        ok,jpeg=cv2.imencode('.jpg',output)
                        if not ok:raise ValueError('Не удалось кодировать кадр')
                        annotated=jpeg.tobytes()
                self.images['rover']={'raw':bytes(data),'cv':annotated}
                info.update(frame=info['frame']+1,received_at=received,error=None,detections=result['detections'],
                            cycle_ms=round((time.monotonic()-started)*1000))
            except Exception as error:
                message=str(error) or type(error).__name__
                if info['error']!=message:self.log('Камера ровера: '+message,'error')
                info['error']=message
            try:await asyncio.wait_for(self.rover_camera_stop.wait(),max(.05,1-(time.monotonic()-started)))
            except TimeoutError:pass

    def state(self):
        return {'token': self.token, 'drones': [{
            'id': key, 'name': spec['name'], 'camera_url': spec['camera'],
            'connected': key in self.robot.clients, 'busy': key in self.manual or self.mission_active(),
            'landing_queued': key in self.land_requested or (self.mission_active() and self.robot.stop_requested.is_set()),
            'camera': self.camera_info[key], 'progress': self.progress.get(key)
        } for key, spec in self.config['drones'].items()],
            'routes': self.routes, 'map': self.map, 'camera_mode': self.camera_mode,
            'mission_active': self.mission_active(),
            'states': self.inventory.states if self.inventory else {},
            'unmatched': self.inventory.unmatched if self.inventory else {},
            'events': list(self.events), 'jobs': list(self.jobs.values())[-20:],
            'rover': {'available': self.rover is not None, 'status': self.rover.status if self.rover else {},
                      'camera': self.rover_camera_info, 'camera_mode': self.rover_camera_mode},
            'ws_port': self.robot.port}


class ObservedServer(WsServer):
    def __init__(self, station):
        super().__init__()
        self.station = station

    async def handler(self, ws):
        # Registry updates remain exclusively owned by the legacy server.
        await super().handler(ws)

    async def transmit(self, msg, addr, timeout=240):
        self.station.progress[addr] = {'command': msg['command'], 'params': msg.get('params', {}), 'status': 'waiting'}
        try:
            result = await super().transmit(msg, addr, timeout)
            self.station.progress[addr]['status'] = 'done'
            self.station.log(f'{addr}: {msg["command"]} подтверждена')
            return result
        except BaseException:
            self.station.progress[addr]['status'] = 'error'
            raise


@web.middleware
async def protect(request, handler):
    station = request.app['station']
    if request.method == 'POST':
        origin = request.headers.get('Origin')
        if origin and origin != f'{request.scheme}://{request.host}':
            raise web.HTTPForbidden(text='Чужой Origin')
        if not hmac.compare_digest(request.headers.get('X-Station-Key', ''), station.token):
            raise web.HTTPForbidden(text='Нет ключа локальной сессии')
    try:
        response = await handler(request)
    except (ValueError, KeyError, TypeError) as error:
        response = web.json_response({'error': str(error)}, status=400)
    response.headers['Cache-Control'] = 'no-store'
    response.headers['X-Content-Type-Options'] = 'nosniff'
    response.headers['Content-Security-Policy'] = "default-src 'self'; img-src 'self' blob:; style-src 'self'; script-src 'self'; frame-ancestors 'none'"
    return response


async def api(request):
    station = request.app['station']
    action = request.match_info['action']
    if request.method == 'GET':
        if action == 'state':
            return web.json_response(station.state())
        if action == 'addresses':
            text = ''.join(f'{k}: {v}\n' for k,v in sorted((station.inventory.states if station.inventory else {}).items()))
            return web.Response(text=text, headers={'Content-Disposition': 'attachment; filename="addresses.txt"'})
        if action == 'image':
            robot_id = request.query.get('id')
            mode = request.query.get('mode', 'cv')
            image = station.images.get(robot_id, {}).get(mode)
            if image is None:
                raise web.HTTPNotFound(text='Снимок ещё не получен')
            return web.Response(body=image, content_type='image/jpeg')
        raise web.HTTPNotFound()
    data = await request.json()
    if not isinstance(data, dict):
        raise ValueError('Ожидается JSON-объект')
    if action == 'rover':
        if station.rover is None:raise ValueError('Ровер не настроен')
        operation=data.get('operation')
        try:
            if operation=='begin':
                result=await station.rover.begin(data['direction'],data.get('speed',.12),data.get('turn_speed',.4))
            elif operation=='pulse':result=await station.rover.pulse(data['lease'])
            elif operation=='stop':result=await station.rover.stop(data.get('lease'))
            else:raise ValueError('Неизвестная команда ровера')
        except ValueError:raise
        except Exception as error:
            return web.json_response({'error':'Ровер: '+(str(error) or type(error).__name__)},status=502)
    elif action == 'rover-camera':
        if data.get('mode')=='off':await station.stop_rover_camera()
        else:await station.start_rover_camera(data['mode'])
        result={'ok':True}
    elif action == 'command':
        result = station.command(data['id'], data['command'], data.get('params', {}))
    elif action == 'mission':
        result = station.start_mission(data['routes'])
    elif action == 'stop':
        result = station.stop_all()
        if station.rover:
            station.launch('Остановка ровера', station.rover.stop())
    elif action == 'routes':
        routes = validate_routes(data['routes'])
        if set(routes) != set(station.config['drones']):
            raise ValueError('Неверные ID маршрутов')
        result = {'routes': routes}
    elif action == 'cameras':
        if data.get('mode') == 'off':
            _, task = station.launch('Остановка камер', station.stop_cameras())
        else:
            await station.start_cameras(data['mode'])
        result = {'ok': True}
    elif action == 'config':
        if station.camera_tasks:
            raise ValueError('Сначала остановите камеры')
        urls = data['cameras']
        if set(urls) != set(station.config['drones']):
            raise ValueError('Неверные ID камер')
        checked = {key: snapshot_url(value) for key, value in urls.items()}
        for value in checked.values():
            parts = urlsplit(value)
            if not parts.hostname or parts.username or parts.password:
                raise ValueError('Неверный адрес камеры')
        for key,value in checked.items():
            station.config['drones'][key]['camera'] = value
        result = {'ok': True}
    elif action == 'reset':
        if station.camera_tasks:
            raise ValueError('Остановите камеры перед новым обходом')
        if station.inventory:
            from scan_two_drones import Inventory
            station.inventory = Inventory(station.map_path)
            station.inventory.save(station.output)
        result = {'ok': True}
    else:
        raise web.HTTPNotFound()
    return web.json_response(result)


def create_app(station, robot_port=8000, robot_host='0.0.0.0', listen_robots=True):
    app = web.Application(middlewares=[protect], client_max_size=128*1024)
    app['station'] = station
    async def lifecycle(app):
        station.session = ClientSession(trust_env=False)
        listener = None
        try:
            if station.rover:await station.rover.start(station.session)
            if listen_robots:
                station.robot.port = robot_port
                listener = await serve(station.robot.handler, robot_host, robot_port, ping_interval=2, ping_timeout=5)
            yield
        finally:
            station.robot.stop_requested.set()
            station.camera_stop.set()
            if station.rover:await station.rover.close()
            await station.stop_rover_camera()
            await station.stop_cameras()
            await station.robot.stop_mission()
            for robot_id in station.manual:
                station.land_requested.add(robot_id)
            await asyncio.gather(*list(station.manual.values()), return_exceptions=True)
            if listener:
                listener.close()
                await listener.wait_closed()
            await station.session.close()
    app.cleanup_ctx.append(lifecycle)
    app.router.add_get('/', lambda request: web.FileResponse(STATIC/'index.html'))
    app.router.add_get('/api/{action}', api)
    app.router.add_post('/api/{action}', api)
    app.router.add_static('/static/', STATIC)
    return app


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--robot-host', default='0.0.0.0')
    parser.add_argument('--robot-port', type=int, default=8000)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    station = Station(config)
    web.run_app(create_app(station, args.robot_port, args.robot_host), host=args.host, port=args.port,
                shutdown_timeout=300)


if __name__ == '__main__':
    main()
