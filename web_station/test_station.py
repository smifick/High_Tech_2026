import asyncio
import copy
import json
from pathlib import Path
import tempfile
import unittest

from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer
from websockets.asyncio.client import connect
from websockets.asyncio.server import serve
from server import Station, create_app, ROOT

CONFIG = json.loads(Path(__file__).with_name('config.json').read_text())
CONFIG.pop('rover_inspection', None)
CONFIG.pop('rover', None)  # Tests must never contact physical hardware


class StationTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.station = Station(copy.deepcopy(CONFIG), Path(self.folder.name)/'addresses.txt')
        self.client = TestClient(TestServer(create_app(self.station, listen_robots=False)))
        await self.client.start_server()
        self.robots = []
        self.listener = await serve(self.station.robot.handler, '127.0.0.1', 0)
        self.uri = f'ws://127.0.0.1:{self.listener.sockets[0].getsockname()[1]}'
        self.messages = []
        self.gate = asyncio.Event(); self.gate.set()
        self.camera_server = None

    async def asyncTearDown(self):
        self.gate.set()
        await self.client.close()
        self.listener.close(); await self.listener.wait_closed()
        await asyncio.gather(*self.robots, return_exceptions=True)
        if self.camera_server:
            await self.camera_server.close()
        self.folder.cleanup()

    async def post(self, action, data=None):
        return await self.client.post('/api/'+action, json=data or {}, headers={'X-Station-Key':self.station.token})

    async def pair(self):
        async def robot(robot_id):
            async with connect(self.uri) as ws:
                await ws.send(robot_id)
                self.assertEqual(json.loads(await ws.recv()), {'type':'registered'})
                async for raw in ws:
                    msg = json.loads(raw);self.messages.append((robot_id, msg))
                    await self.gate.wait()
                    await ws.send(json.dumps({'type':'result','command':msg['command'],'ok':True}))
        self.robots = [asyncio.create_task(robot(key)) for key in CONFIG['drones']]
        for _ in range(100):
            if len(self.station.robot.clients)==2:return
            await asyncio.sleep(.01)
        self.fail('Clients did not register')

    async def done(self):
        for _ in range(200):
            if not self.station.manual and not self.station.mission_active():return
            await asyncio.sleep(.01)
        self.fail('Commands did not finish')

    async def test_page_load_has_no_side_effects(self):
        await self.pair()
        response=await self.client.get('/')
        self.assertEqual(response.status,200)
        self.assertIn('Полёт и камеры',await response.text())
        response=await self.client.get('/api/state');state=await response.json()
        self.assertEqual(len(state['drones']),2)
        self.assertEqual(state['camera_mode'],'off')
        self.assertEqual(self.messages,[])
        self.assertEqual(self.station.camera_tasks,{})

    async def test_cancelled_flight_never_starts_inspection(self):
        from unittest.mock import AsyncMock, patch
        self.station.robot.run_mission = AsyncMock(return_value=False)
        with patch('server.asyncio.create_subprocess_exec', new_callable=AsyncMock) as spawn:
            await self.station.flight_and_inspection(self.station.routes)
            spawn.assert_not_awaited()

    async def test_disconnected_rejected(self):
        response=await self.post('command',{'id':'drone3','command':'navigate','params':{'x':0,'y':0,'z':1}})
        self.assertEqual(response.status,400)

    async def test_csrf_rejected(self):
        response=await self.client.post('/api/stop',json={})
        self.assertEqual(response.status,403)
        response=await self.client.post('/api/stop',json={},headers={'X-Station-Key':self.station.token,'Origin':'https://foreign.example'})
        self.assertEqual(response.status,403)

    async def test_manual_busy_then_land(self):
        await self.pair();self.gate.clear()
        response=await self.post('command',{'id':'drone3','command':'navigate','params':{'x':1,'y':1,'z':1.7}})
        self.assertEqual(response.status,200)
        response=await self.post('command',{'id':'drone3','command':'navigate','params':{'x':1,'y':1,'z':1.7}})
        self.assertEqual(response.status,400)
        response=await self.post('command',{'id':'drone3','command':'land'})
        self.assertEqual(response.status,200)
        self.gate.set();await self.done()
        self.assertEqual([m['command'] for _,m in self.messages],['navigate','land'])
        self.assertEqual(self.messages[0][1]['params']['frame_id'],'aruco_map')

    async def test_nonfinite_coordinate_rejected(self):
        await self.pair()
        response=await self.post('command',{'id':'drone3','command':'navigate','params':{'x':float('nan'),'y':0,'z':1}})
        self.assertEqual(response.status,400)
        self.assertEqual(self.messages,[])

    async def test_mission_stop_both(self):
        await self.pair();self.gate.clear()
        response=await self.post('mission',{'routes':self.station.routes})
        self.assertEqual(response.status,200)
        for _ in range(100):
            if len(self.messages)>=2:break
            await asyncio.sleep(.01)
        response=await self.post('stop')
        self.assertEqual(response.status,200)
        self.gate.set();await self.done()
        for key in CONFIG['drones']:
            self.assertEqual([m['command'] for peer,m in self.messages if peer==key],['navigate','land'])

    async def test_intersecting_routes_rejected(self):
        await self.pair()
        bad={key:{'points':[[0,0],[1,1]]} for key in CONFIG['drones']}
        response=await self.post('mission',{'routes':bad})
        self.assertEqual(response.status,400)
        self.assertEqual(self.messages,[])

    async def test_raw_camera_and_stop(self):
        jpeg=(ROOT/'YOLO/assets/test_match_1.png').read_bytes()
        import cv2,numpy as np
        jpeg=cv2.imencode('.jpg',cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR))[1].tobytes()
        app=web.Application()
        app.router.add_get('/snapshot',lambda request:web.Response(body=jpeg,content_type='image/jpeg'))
        self.camera_server=TestServer(app);await self.camera_server.start_server()
        for spec in self.station.config['drones'].values():spec['camera']=str(self.camera_server.make_url('/snapshot'))
        self.assertEqual((await self.post('cameras',{'mode':'raw'})).status,200)
        for _ in range(100):
            if len(self.station.images)==2:break
            await asyncio.sleep(.02)
        self.assertEqual(len(self.station.images),2)
        response=await self.client.get('/api/image?id=drone3')
        self.assertEqual(response.status,200)
        self.assertEqual(response.content_type,'image/jpeg')
        self.assertEqual(self.station.processor,None)
        self.assertEqual(self.messages,[])
        await self.station.stop_cameras()
        self.assertEqual(self.station.camera_mode,'off')

    async def test_reset_export(self):
        from scan_two_drones import Inventory
        self.station.inventory=Inventory(self.station.map_path)
        self.station.inventory.states={100:'занято',101:'аномалия'}
        response=await self.client.get('/api/addresses')
        self.assertEqual(await response.text(),'100: занято\n101: аномалия\n')
        self.assertEqual((await self.post('reset')).status,200)
        self.assertEqual(self.station.inventory.states,{})


if __name__=='__main__':unittest.main()
