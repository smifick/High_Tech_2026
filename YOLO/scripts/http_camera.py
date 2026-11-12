"""Bounded MJPEG reader for the existing ROS web_video_server."""
import asyncio
from contextlib import asynccontextmanager
import urllib.request


class MJPEGStream:
    def __init__(self, response):
        self.response = response
        self.frames = asyncio.Queue(maxsize=1)
        self.task = asyncio.create_task(self.read())

    def publish(self, value):
        if self.frames.full():
            self.frames.get_nowait()
        self.frames.put_nowait(value)

    async def read(self):
        buffer = b''
        try:
            while True:
                chunk = await asyncio.to_thread(self.response.read1, 65536)
                if not chunk:
                    raise ConnectionError('HTTP-видеопоток завершился')
                buffer += chunk
                while True:
                    start = buffer.find(b'\xff\xd8')
                    if start < 0:
                        buffer = buffer[-1:]
                        break
                    buffer = buffer[start:]
                    end = buffer.find(b'\xff\xd9', 2)
                    if end < 0:
                        break
                    self.publish(buffer[:end + 2])
                    buffer = buffer[end + 2:]
                if len(buffer) > 16 * 1024 * 1024:
                    raise ValueError('Слишком большой или повреждённый JPEG')
        except asyncio.CancelledError:
            raise
        except Exception as error:
            self.publish(error)

    async def send(self, message):
        pass  # HTTP server pushes frames without per-frame requests.

    async def recv(self):
        result = await self.frames.get()
        if isinstance(result, Exception):
            raise result
        return result


@asynccontextmanager
async def connect_mjpeg(url):
    # No system HTTP proxy for the drone's LAN address.
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    response = await asyncio.to_thread(opener.open, url, timeout=5)
    stream = None
    try:
        if 'multipart/' not in response.headers.get('Content-Type', '').lower():
            raise ValueError('Нужен URL /stream?topic=..., а не страница /stream_viewer')
        stream = MJPEGStream(response)
        yield stream
    finally:
        if stream:
            stream.task.cancel()
            await asyncio.gather(stream.task, return_exceptions=True)
        await asyncio.to_thread(response.close)
