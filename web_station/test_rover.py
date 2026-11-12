import asyncio
import time
import unittest
from aiohttp import web, ClientSession
from aiohttp.test_utils import TestServer
from rover_adapter import RoverAdapter

class RoverTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.commands=[]
        async def status(request):return web.json_response({'ok':True,'drive_type':'mecanum','timeout_sec':.25})
        async def command(request):
            self.commands.append(await request.json());return web.json_response({'ok':True})
        app=web.Application();app.router.add_get('/api/drive',status)
        app.router.add_post('/api/drive/command',command);app.router.add_post('/api/drive/stop',command)
        self.server=TestServer(app);await self.server.start_server()
        self.session=ClientSession();self.adapter=RoverAdapter(str(self.server.make_url('/')),lambda *a:None)
        await self.adapter.start(self.session)
        for _ in range(50):
            if self.adapter.status['connected']:break
            await asyncio.sleep(.01)
    async def asyncTearDown(self):
        await self.adapter.close();await self.session.close();await self.server.close()
    async def test_connect_does_not_move(self):
        self.assertTrue(self.adapter.status['connected']);self.assertEqual(self.commands,[])
    async def test_hold_and_stop_rejects_late_pulse(self):
        lease=(await self.adapter.begin('forward'))['lease']
        await self.adapter.pulse(lease)
        self.assertEqual(self.commands[-1],{'linear_x':.12,'linear_y':0.,'angular_z':0.})
        await self.adapter.stop(lease)
        with self.assertRaises(ValueError):await self.adapter.pulse(lease)
        self.assertEqual(self.commands[-1],{})
    async def test_no_background_motor_heartbeat(self):
        lease=(await self.adapter.begin('left'))['lease']
        await asyncio.sleep(.3)
        self.assertEqual(len(self.commands),1)
        self.adapter.last_pulse=time.monotonic()-1
        with self.assertRaises(ValueError):await self.adapter.pulse(lease)
        self.assertEqual(len(self.commands),1)
    async def test_invalid_speed_and_direction(self):
        for value in [float('nan'),True,-1,1]:
            with self.assertRaises(ValueError):await self.adapter.begin('forward',value)
        with self.assertRaises(ValueError):await self.adapter.begin('unknown')
        self.adapter.status['drive']['drive_type']='differential'
        with self.assertRaises(ValueError):await self.adapter.begin('left')
        self.assertEqual(self.commands,[])
    async def test_old_release_cannot_stop_new_hold(self):
        lease=(await self.adapter.begin('forward'))['lease'];await self.adapter.stop(lease)
        new=(await self.adapter.begin('backward'))['lease']
        await self.adapter.stop(lease)
        self.assertEqual(self.adapter.lease,new)
        self.assertEqual(self.commands[-1]['linear_x'],-.12)

    async def test_status_timeout_grace_and_recovery(self):
        from unittest.mock import AsyncMock
        self.adapter.stop_event.set()
        await self.adapter.poll_task
        original = self.adapter.request
        self.adapter.request = AsyncMock(side_effect=TimeoutError())
        await self.adapter.poll_once()
        self.assertTrue(self.adapter.status['connected'])
        self.assertTrue(self.adapter.status['degraded'])
        self.adapter.last_status_ok = time.monotonic()-7
        await self.adapter.poll_once()
        self.assertFalse(self.adapter.status['connected'])
        with self.assertRaises(ValueError):await self.adapter.begin('forward')
        self.adapter.request = original
        await self.adapter.poll_once()
        self.assertTrue(self.adapter.status['connected'])
        self.assertFalse(self.adapter.status['degraded'])
        self.assertEqual(self.commands, [])

    async def test_first_status_failure_is_not_connected(self):
        from unittest.mock import AsyncMock
        adapter = RoverAdapter('http://unused', lambda *a: None)
        adapter.request = AsyncMock(side_effect=TimeoutError())
        await adapter.poll_once()
        self.assertFalse(adapter.status['connected'])
