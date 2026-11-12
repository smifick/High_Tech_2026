"""Adapter for stock Sverk Rover Web. No motor commands on connect."""
import asyncio
import math
import secrets
import time
from aiohttp import ClientTimeout

DIRECTIONS = {'forward':(1,0,0), 'backward':(-1,0,0), 'left':(0,1,0),
              'right':(0,-1,0), 'turn_left':(0,0,1), 'turn_right':(0,0,-1)}


class RoverAdapter:
    def __init__(self, base_url, log):
        self.base = base_url.rstrip('/')
        self.log = log
        self.session = None
        self.io_lock = asyncio.Lock()
        self.lease = None
        self.last_pulse = 0.
        self.command = None
        self.poll_task = None
        self.stop_event = asyncio.Event()
        self.status = {'connected':False,'error':None,'checked_at':None,'drive':{}}

    async def request(self, path, payload=None):
        if self.session is None:
            raise ValueError('Адаптер ровера ещё не подключён')
        kwargs={'timeout':ClientTimeout(total=1.5)}
        if payload is not None:kwargs['json']=payload
        async with self.session.request('GET' if payload is None else 'POST', self.base+path, **kwargs) as response:
            response.raise_for_status()
            result=await response.json()
            if result.get('ok') is not True:
                raise ValueError(result.get('error','Ровер отклонил запрос'))
            return result

    async def poll(self):
        while not self.stop_event.is_set():
            try:
                drive=await self.request('/api/drive')
                self.status.update(connected=True,error=None,checked_at=time.time(),drive=drive)
            except Exception as error:
                self.status.update(connected=False,error=str(error) or type(error).__name__)
            try:await asyncio.wait_for(self.stop_event.wait(),2)
            except TimeoutError:pass

    async def start(self, session):
        self.session=session
        self.stop_event.clear()
        self.poll_task=asyncio.create_task(self.poll())

    async def begin(self, direction, speed=.12, turn_speed=.4):
        if direction not in DIRECTIONS:raise ValueError('Неизвестное направление')
        for value, maximum in ((speed,.2),(turn_speed,.7)):
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not 0 < value <= maximum:
                raise ValueError('Скорость должна быть 0..0.2 м/с; поворот 0..0.7 рад/с')
        if not self.status['connected'] or time.time()-(self.status['checked_at'] or 0)>5:
            raise ValueError('Нет свежего статуса ровера')
        if direction in ('left','right') and self.status['drive'].get('drive_type')!='mecanum':
            raise ValueError('Боковой ход доступен только для mecanum')
        if self.lease and time.monotonic()-self.last_pulse<.6:
            raise ValueError('Управление уже удерживается другим запросом')
        x,y,z=DIRECTIONS[direction]
        self.command={'linear_x':x*speed,'linear_y':y*speed,'angular_z':z*turn_speed}
        self.lease=secrets.token_urlsafe(16)
        lease=self.lease
        self.last_pulse=time.monotonic()
        await self.pulse(lease)
        self.log('Ровер: '+direction)
        return {'lease':lease}

    async def pulse(self, lease):
        async with self.io_lock:
            if not lease or lease != self.lease or time.monotonic()-self.last_pulse>.6:
                raise ValueError('Сессия движения истекла. Отпустите и снова удерживайте кнопку.')
            try:
                result=await self.request('/api/drive/command',self.command)
            except Exception:
                self.lease=None
                raise
            self.last_pulse=time.monotonic()
            return result

    async def stop(self, lease=None):
        if lease is not None and lease != self.lease:
            return {'ok':True,'obsolete':True}
        self.lease=None
        async with self.io_lock:
            result=await self.request('/api/drive/stop',{})
        self.log('Ровер: остановка подтверждена')
        return result

    async def close(self):
        self.stop_event.set()
        if self.lease is not None:
            try:await self.stop()
            except Exception as error:self.log(f'Ровер: остановка не подтверждена — {error}','error')
        if self.poll_task:await self.poll_task
