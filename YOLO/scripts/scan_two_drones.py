"""Merge one-second snapshots from two drones into an ArUco inventory."""
from collections import deque
import argparse
import csv
import io
import asyncio
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
from box_detector import BoxProcessor, DEFAULT_BOX_WEIGHTS
from snapshot_camera import SnapshotCamera

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MAP = ROOT / 'drone/aruco_nav/aruco_map_750x450_id100.txt'
PRIORITY = {'пусто': 0, 'занято': 1, 'аномалия': 2}


class Inventory:
    def __init__(self, map_path):
        self.expected = set()
        self.coordinates = {}
        for line in Path(map_path).read_text().splitlines():
            if line.strip() and not line.lstrip().startswith('#'):
                marker = int(line.split()[0])
                if marker in self.expected:
                    raise ValueError(f'Повтор ID в карте: {marker}')
                self.expected.add(marker)
                values = line.split()
                self.coordinates[marker] = [float(v) for v in values[2:5]]
        if not self.expected:
            raise ValueError('Карта не содержит меток')
        self.states = {}
        self.votes = {}
        self.unmatched = {}
        self.seen_by = {}
        self.unknown = set()
        self.sources = {}

    def update(self, source, result):
        visible = {int(key) for key in result['markers']}
        self.unknown.update(visible - self.expected)
        current = {}
        self.unmatched[source] = [box for box in result['detections'] if box['marker_id'] is None]
        # At most one vote per address per frame, chosen by confidence, not defect priority.
        for box in result['detections']:
            marker = box['marker_id']
            if marker in visible & self.expected:
                confidence = float(box.get('confidence', 1.0))
                if marker not in current or confidence > current[marker][1]:
                    current[marker] = ('аномалия' if box['anomaly'] else 'занято', confidence)
        for marker in visible & self.expected:
            self.seen_by.setdefault(marker, set()).add(source)
            if marker in current:
                votes = self.votes.setdefault(marker, deque(maxlen=9))
                votes.append(current[marker])
                scores = {state: sum(weight for label, weight in votes if label == state)
                          for state in ('занято', 'аномалия')}
                # A tie preserves the previous state; a single defect is no longer permanent.
                if scores['занято'] != scores['аномалия']:
                    self.states[marker] = max(scores, key=scores.get)
                else:
                    self.states.setdefault(marker, current[marker][0])
            elif marker not in self.states:
                self.states[marker] = 'пусто'
        self.sources[source] = {'last_success': time.time(), 'visible_ids': sorted(visible), 'error': None}

    def save(self, output):
        output = Path(output)
        output.parent.mkdir(parents=True, exist_ok=True)
        def atomic(path, text):
            temporary = path.with_name(path.name + '.tmp')
            temporary.write_text(text, encoding='utf-8')
            temporary.replace(path)
        atomic(output, ''.join(f'{marker}: {self.states[marker]}\n' for marker in sorted(self.states)))
        report = {'expected_ids': sorted(self.expected), 'read_ids': sorted(self.states),
                  'unread_ids': sorted(self.expected - self.states.keys()),
                  'coordinates_m': self.coordinates, 'updated_at': time.time(),
                  'unknown_ids': sorted(self.unknown), 'complete': self.expected == self.states.keys(),
                  'seen_by': {key: sorted(value) for key, value in self.seen_by.items()}, 'sources': self.sources,
                  'unmatched_boxes': self.unmatched,
                  'classification_evidence': {marker: {
                      'observations': len(votes),
                      'normal_score': sum(weight for label, weight in votes if label == 'занято'),
                      'anomaly_score': sum(weight for label, weight in votes if label == 'аномалия')
                  } for marker, votes in self.votes.items()}}
        atomic(output.with_suffix('.coverage.json'), json.dumps(report, ensure_ascii=False, indent=2))
        table = io.StringIO()
        writer = csv.writer(table)
        writer.writerow(['marker_id', 'state', 'x_m', 'y_m', 'z_m', 'sources'])
        for marker in sorted(self.expected):
            writer.writerow([marker, self.states.get(marker, 'не осмотрено'),
                             *self.coordinates[marker], ';'.join(sorted(self.seen_by.get(marker, [])))])
        atomic(output.with_name('inventory.csv'), table.getvalue())


async def scan(args):
    inventory = Inventory(args.map)
    processor = BoxProcessor(args.weights, args.device, args.conf, args.dictionary, args.match_angle)
    cameras = [SnapshotCamera(url) for url in args.servers]
    inventory.save(args.output)
    rounds = 0
    try:
        while not args.rounds or rounds < args.rounds:
            started = time.monotonic()
            photos = await asyncio.gather(*(camera.recv() for camera in cameras), return_exceptions=True)
            for url, photo in zip(args.servers, photos):
                if isinstance(photo, Exception):
                    inventory.sources.setdefault(url, {})['error'] = str(photo)
                    print(f'{url}: ошибка снимка: {photo}', flush=True)
                    continue
                frame = cv2.imdecode(np.frombuffer(photo, np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    inventory.sources.setdefault(url, {})['error'] = 'Некорректный JPEG'
                    continue
                # One model instance, serialized inference; camera requests run concurrently.
                output, result = await asyncio.to_thread(processor.process, frame)
                inventory.update(url, result)
                with args.output.with_name('observations.jsonl').open('a', encoding='utf-8') as journal:
                    journal.write(json.dumps({'timestamp': time.time(), 'source': url, **result}, ensure_ascii=False) + '\n')
                unmatched = len(inventory.unmatched[url])
                print(f'{url}: найдено {len(result["detections"])}; без метки {unmatched}', flush=True)
                if args.save_frames:
                    folder = args.save_frames / f'drone{args.servers.index(url)+1}'
                    folder.mkdir(parents=True, exist_ok=True)
                    name = str(time.time_ns())
                    for suffix, image in (('raw', frame), ('result', output)):
                        if not cv2.imwrite(str(folder / f'{name}_{suffix}.jpg'), image):
                            raise RuntimeError('Не удалось сохранить диагностический кадр')
                    (folder / f'{name}.json').write_text(json.dumps(result, ensure_ascii=False), encoding='utf-8')
                if not args.headless:
                    cv2.imshow(url, output)
            inventory.save(args.output)
            print(f'Считано {len(inventory.states)}/{len(inventory.expected)}; файл: {args.output}', flush=True)
            rounds += 1
            if not args.headless and cv2.waitKey(1) & 0xff in (27, ord('q')):
                break
            if args.rounds and rounds >= args.rounds:
                break
            while time.monotonic() - started < args.interval:
                if not args.headless and cv2.waitKey(1) & 0xff in (27, ord('q')):
                    return
                await asyncio.sleep(.03)
    finally:
        inventory.save(args.output)
        if not args.headless:
            cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--servers', nargs=2, required=True)
    parser.add_argument('--map', type=Path, default=DEFAULT_MAP)
    parser.add_argument('--weights', type=Path, default=DEFAULT_BOX_WEIGHTS)
    parser.add_argument('--dictionary', default='DICT_4X4_1000')
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--match-angle', type=int, choices=(0,90,180,270), default=90)
    parser.add_argument('--conf', type=float, default=.25)
    parser.add_argument('--interval', type=float, default=1)
    parser.add_argument('--rounds', type=int, default=0, help='0 — до Ctrl+C')
    parser.add_argument('--output', type=Path, default=ROOT/'runs/addresses.txt')
    parser.add_argument('--save-frames', type=Path, help='Сохранять исходные фото, разметку и JSON для проверки ошибок')
    parser.add_argument('--headless', action='store_true')
    args = parser.parse_args()
    if len(set(args.servers)) != 2 or not math.isfinite(args.interval) or args.interval <= 0 or args.rounds < 0 or not 0 <= args.conf <= 1:
        parser.error('Нужны два разных адреса, interval > 0, rounds >= 0, conf от 0 до 1')
    try:
        asyncio.run(scan(args))
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
