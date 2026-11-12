"""Request individual JPEG photos from the existing ROS web_video_server."""
import asyncio
from contextlib import asynccontextmanager
from urllib.parse import urlsplit, urlunsplit, parse_qsl, urlencode
import urllib.request


def snapshot_url(url):
    parts = urlsplit(url)
    if parts.scheme not in ('http', 'https'):
        raise ValueError('Для фото нужен HTTP URL штатной камеры на порту 8080')
    if parts.path not in ('', '/', '/stream', '/stream_viewer', '/snapshot'):
        raise ValueError('Ожидается адрес камеры с путём /snapshot или /stream')
    query = dict(parse_qsl(parts.query))
    query.setdefault('topic', '/camera_1/image_raw')
    query.pop('type', None)  # Legacy servers select JPEG snapshots automatically.
    return urlunsplit((parts.scheme, parts.netloc, '/snapshot', urlencode(query, safe='/'), ''))


class SnapshotCamera:
    def __init__(self, url):
        self.url = snapshot_url(url)
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def fetch(self):
        request = urllib.request.Request(self.url, headers={'Cache-Control': 'no-cache'})
        with self.opener.open(request, timeout=5) as response:
            if 'image/jpeg' not in response.headers.get('Content-Type', '').lower():
                raise ValueError('Камера вернула не JPEG: проверьте URL /snapshot и топик')
            data = response.read(16 * 1024 * 1024 + 1)
        if not data or len(data) > 16 * 1024 * 1024:
            raise ValueError('Пустой или слишком большой снимок')
        return data

    async def send(self, message):
        pass

    async def recv(self):
        return await asyncio.to_thread(self.fetch)


@asynccontextmanager
async def connect_snapshot(url):
    yield SnapshotCamera(url)
