import asyncio
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest

import cv2
from snapshot_camera import snapshot_url, SnapshotCamera
from video_receiver import receive
from box_detector import BoxProcessor

ROOT = Path(__file__).resolve().parents[2]


class Snapshots(unittest.IsolatedAsyncioTestCase):
    def test_url(self):
        url = snapshot_url('http://drone:8080/stream?topic=/camera_1/image_raw&type=mjpeg')
        self.assertIn('/snapshot?', url)
        self.assertNotIn('type=', url)
        self.assertIn('topic=/camera_1/image_raw', url)
        with self.assertRaises(ValueError):
            snapshot_url('ws://drone:8765')

    async def test_photo_each_second_with_model(self):
        frame = cv2.imread(str(ROOT/'YOLO/assets/test_match_1.png'))
        self.assertIsNotNone(frame)
        jpeg = cv2.imencode('.jpg', frame)[1].tobytes()
        requests = []
        async def handler(reader, writer):
            try:
                request = await reader.readuntil(b'\r\n\r\n')
                requests.append((time.monotonic(), request))
                writer.write(b'HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(jpeg)).encode()+b'\r\nConnection: close\r\n\r\n'+jpeg)
                await writer.drain()
            finally:
                writer.close()
                await writer.wait_closed()
        server = await asyncio.start_server(handler, '127.0.0.1', 0)
        processor = BoxProcessor()
        async with server:
            with tempfile.TemporaryDirectory() as directory:
                args = SimpleNamespace(server=f'http://127.0.0.1:{server.sockets[0].getsockname()[1]}/stream?type=mjpeg',
                    capture='snapshot', interval=1., max_frames=3, headless=True, output=None,
                    json_output=Path(directory)/'photos.jsonl')
                self.assertEqual(await receive(args, processor), 3)
                records = [json.loads(line) for line in args.json_output.read_text().splitlines()]
                self.assertEqual(len(records), 3)
                self.assertTrue(all('detections' in record for record in records))
        self.assertEqual(len(requests), 3)
        self.assertTrue(all(b'GET /snapshot?' in raw for _, raw in requests))
        self.assertTrue(all(b'topic=/camera_1/image_raw' in raw and b'type=' not in raw for _, raw in requests))
        self.assertTrue(all(b'/stream?' not in raw for _, raw in requests))
        for previous, current in zip(requests, requests[1:]):
            self.assertGreaterEqual(current[0]-previous[0], .95)


if __name__ == '__main__':
    unittest.main()
