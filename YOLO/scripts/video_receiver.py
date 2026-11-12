"""Run on laptop: receive JPEG frames and process with mathc_aruco_box."""
import argparse
import asyncio
import json
import math
from urllib.error import URLError, HTTPError
from http.client import RemoteDisconnected
from pathlib import Path
import time

import cv2
import numpy as np
from websockets.asyncio.client import connect
from http_camera import connect_mjpeg
from snapshot_camera import connect_snapshot, snapshot_url
from box_detector import BoxProcessor, DEFAULT_BOX_WEIGHTS
from mathc_aruco_box import FrameProcessor, DEFAULT_WEIGHTS


async def receive(args, processor):
    count = 0
    errors = 0
    record = None
    try:
        if args.json_output:
            args.json_output.parent.mkdir(parents=True, exist_ok=True)
            record = args.json_output.open('w', encoding='utf-8')
        interval = getattr(args, 'interval', 0) if getattr(args, 'capture', 'stream') == 'snapshot' else 0
        next_request = 0.
        connection = (connect_mjpeg(args.server) if args.server.startswith(('http://', 'https://'))
                      else connect(args.server, compression=None, max_size=16*1024*1024,
                                   max_queue=1, ping_interval=5, ping_timeout=10))
        if getattr(args, 'capture', 'stream') == 'snapshot':
            connection = connect_snapshot(args.server)
        async with connection as ws:
            while not args.max_frames or count < args.max_frames:
                # Start requests at least interval seconds apart; never queue missed photos.
                while time.monotonic() < next_request:
                    if not args.headless and cv2.waitKey(1) & 0xff in (27, ord('q')):
                        return count
                    await asyncio.sleep(min(.03, max(0, next_request - time.monotonic())))
                started = time.monotonic()
                next_request = started + interval
                # Request only after processing: slow inference cannot build a video backlog.
                await ws.send('next')
                raw = await asyncio.wait_for(ws.recv(), timeout=10)
                if isinstance(raw, str):
                    errors += 1
                    print(f'Кадр недоступен: {raw}', flush=True)
                    if errors >= 10:
                        raise RuntimeError('10 последовательных ошибок камеры')
                    await asyncio.sleep(.2)
                    continue
                frame = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    raise ValueError('Получен некорректный JPEG')
                errors = 0
                received_at = time.time()
                output, detections = await asyncio.to_thread(processor.process, frame)
                count += 1
                detections.update(frame=count, received_at=received_at, cycle_ms=round((time.monotonic()-started)*1000, 1))
                line = json.dumps(detections, ensure_ascii=False)
                print(line, flush=True)
                if record:
                    record.write(line + '\n')
                    record.flush()
                if args.output:
                    args.output.parent.mkdir(parents=True, exist_ok=True)
                    if not cv2.imwrite(str(args.output), output):
                        raise RuntimeError(f'Не удалось сохранить {args.output}')
                if not args.headless:
                    cv2.imshow('Drone: CV', output)
                    if cv2.waitKey(1) & 0xff in (27, ord('q')):
                        break
    finally:
        if record:
            record.close()
        if not args.headless:
            cv2.destroyAllWindows()
    if not count:
        raise RuntimeError('Не обработано ни одного кадра')
    return count


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--server', required=True, help='http://IP_ДРОНА:8080/stream?topic=/camera_1/image_raw&type=mjpeg или ws://IP:8765')
    parser.add_argument('--capture', choices=('snapshot', 'stream'), default='snapshot', help='По умолчанию отдельные фотографии')
    parser.add_argument('--interval', type=float, default=1.0, help='Интервал между запросами фото в секундах')
    parser.add_argument('--mode', choices=('boxes', 'aruco'), default='boxes', help='boxes — боксы/аномалии; aruco — прежнее сопоставление с метками')
    parser.add_argument('--weights', type=Path)
    parser.add_argument('--device', default='cpu', help='cpu или 0 для GPU')
    parser.add_argument('--dictionary', default='DICT_4X4_250')
    parser.add_argument('--conf', type=float, default=.25)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--max-frames', type=int, default=0, help='0 — без ограничения')
    parser.add_argument('--output', type=Path, help='Перезаписывать последний обработанный кадр')
    parser.add_argument('--json-output', type=Path, help='Сохранить результаты в JSONL')
    args = parser.parse_args()
    if args.max_frames < 0 or not 0 <= args.conf <= 1:
        parser.error('Некорректные max-frames/conf')
    if not math.isfinite(args.interval) or args.interval <= 0:
        parser.error('interval должен быть положительным конечным числом')
    if args.capture == 'snapshot':
        try:
            args.server = snapshot_url(args.server)
        except ValueError as error:
            parser.error(str(error))
        print(f'Фото каждые {args.interval:g} с: {args.server}', flush=True)
    if args.mode == 'boxes':
        processor = BoxProcessor(args.weights or DEFAULT_BOX_WEIGHTS, args.device, args.conf, args.dictionary)
    else:
        processor = FrameProcessor(args.weights or DEFAULT_WEIGHTS, args.device, args.dictionary, args.conf)
    try:
        asyncio.run(receive(args, processor))
    except HTTPError as error:
        parser.exit(1, f'Камера ответила HTTP {error.code}: {args.server}. Проверьте /snapshot и имя топика на главной странице сервиса.\n')
    except URLError as error:
        parser.exit(1, f'Ошибка доступа к {args.server}: {error.reason}. Проверьте IP, сеть и доступность порта с ноутбука.\n')
    except RemoteDisconnected:
        parser.exit(1, f'Сервис {args.server} закрыл соединение без HTTP-ответа. Проверьте главную страницу порта 8080 и логи web_video_server; это ещё не ошибка модели.\n')
    except TimeoutError:
        parser.exit(1, f'Истекло время ожидания снимка: {args.server}. Проверьте, публикуется ли топик камеры.\n')
    except ConnectionRefusedError:
        parser.exit(1, f"Соединение отклонено: {args.server}. На дроне должна быть строка LISTEN; проверьте IP и публикацию порта из URL в Docker.\n")
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
