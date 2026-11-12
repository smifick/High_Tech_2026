import argparse
import asyncio
import json
from pathlib import Path
import sys
import tempfile
import unittest

from aiohttp.test_utils import TestClient, TestServer
from websockets.asyncio.client import connect
from websockets.asyncio.server import serve

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'YOLO/scripts'))
from station.__main__ import Station, WsServer
from mission import load_mission
from scan_two_drones import Inventory, DEFAULT_MAP
from field_report import score
from aiohttp import web


class ArtifactTests(unittest.TestCase):
    def test_inventory_csv_includes_unseen_and_map_coordinates(self):
        with tempfile.TemporaryDirectory() as directory:
            inv = Inventory(DEFAULT_MAP)
            inv.update('drone1', {'markers': {100: []}, 'detections': []})
            inv.save(Path(directory)/'addresses.txt')
            import csv
            with (Path(directory)/'inventory.csv').open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 45)
            self.assertEqual(rows[0]['state'], 'пусто')
            self.assertEqual(rows[0]['x_m'], '-3.0')
            self.assertEqual(rows[-1]['state'], 'не осмотрено')

    def test_duplicate_prediction_and_wrong_class_are_false_positives(self):
        pred = lambda name: {'confidence': .9, 'class_name': name, 'xyxy': [0,0,10,10]}
        truth = [{'class_name':'box_hitech','xyxy':[0,0,10,10]}]
        self.assertEqual(score([pred('box_hitech')]*2, truth), {'tp':1,'fp':1,'fn':0})
        self.assertEqual(score([pred('box_hitech_defect')], truth), {'tp':0,'fp':1,'fn':1})


class FlightTests(unittest.IsolatedAsyncioTestCase):
    async def test_two_routes_and_landing_over_real_websockets(self):
        station = WsServer('127.0.0.1', 0)
        routes = load_mission(ROOT/'drone/aruco_nav/routes.example.json')
        events = {robot: [] for robot in routes}
        async def robot(name, port):
            async with connect(f'ws://127.0.0.1:{port}') as ws:
                await ws.send(name)
                self.assertEqual(json.loads(await ws.recv()), {'type':'registered'})
                async for raw in ws:
                    msg = json.loads(raw)
                    events[name].append(msg['command'])
                    await ws.send(json.dumps({'type':'result','command':msg['command'],'ok':True}))
        async with serve(station.handler, '127.0.0.1', 0) as listener:
            port = listener.sockets[0].getsockname()[1]
            clients = [asyncio.create_task(robot(name, port)) for name in routes]
            try:
                async with asyncio.timeout(5):
                    while len(station.clients) != 2:
                        await asyncio.sleep(.01)
                    await station.run_mission(routes)
                for name, spec in routes.items():
                    self.assertEqual(events[name], ['navigate']*len(spec['points'])+['land'])
            finally:
                for client in clients:
                    client.cancel()
                await asyncio.gather(*clients, return_exceptions=True)

    async def test_timeout_disconnects_and_cannot_acknowledge_next_command(self):
        station = WsServer('127.0.0.1', 0)
        async with serve(station.handler, '127.0.0.1', 0) as listener:
            port = listener.sockets[0].getsockname()[1]
            async with connect(f'ws://127.0.0.1:{port}') as ws:
                await ws.send('test')
                await ws.recv()
                with self.assertRaises(TimeoutError):
                    await station.transmit({'command':'land'}, 'test', timeout=.05)
                await ws.wait_closed()
                await asyncio.sleep(.01)
                self.assertNotIn('test', station.clients)


class WebTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        args = argparse.Namespace(routes=ROOT/'drone/aruco_nav/routes.example.json', output=Path(self.tmp.name),
                                  ws_host='127.0.0.1', ws_port=0)
        self.station = Station(args)
        app = web.Application()
        app.add_routes([web.get('/',self.station.index), web.get('/api/state',self.station.state),
                        web.post('/api/{action}', self.station.action), web.get('/download/{name}', self.station.download)])
        self.client = TestClient(TestServer(app))
        await self.client.start_server()

    async def asyncTearDown(self):
        await self.client.close()
        self.tmp.cleanup()

    async def test_ui_and_state_without_connected_hardware(self):
        response = await self.client.get('/')
        self.assertIn('Инвентаризация', await response.text())
        response = await self.client.get('/api/state')
        data = await response.json()
        self.assertEqual(data['connected'], [])
        self.assertEqual(data['scanner'], 'остановлен')
        self.assertIsNone(data['coverage'])

    async def test_start_rejected_without_both_drones_or_from_foreign_site(self):
        response = await self.client.post('/api/start')
        self.assertEqual(response.status, 403)
        response = await self.client.post('/api/start', headers={'X-Station-Action':'1'})
        self.assertEqual(response.status, 409)
        response = await self.client.post('/api/start', headers={'X-Station-Action':'1','Sec-Fetch-Site':'cross-site'})
        self.assertEqual(response.status, 403)

    async def test_only_allowlisted_files_downloaded(self):
        (self.station.attempt/'inventory.csv').write_text('marker_id,state\n100,пусто\n')
        response = await self.client.get('/download/inventory.csv')
        self.assertEqual(response.status, 200)
        self.assertIn('100,пусто', await response.text())
        response = await self.client.get('/download/scan.log')
        self.assertEqual(response.status, 404)
