"""Offline frames -> ArUco addresses -> YOLO states -> common attempt report."""
import argparse
import csv
import json
from pathlib import Path
from datetime import datetime, timezone
from logic import CLASSES, classify, summarize


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--weights', required=True, help='Trained three-class best.pt')
    p.add_argument('--source', required=True, help='Image or folder with frames')
    p.add_argument('--config', default='config.json')
    p.add_argument('--attempt', required=True, help='New directory for each attempt')
    p.add_argument('--view', choices=['top', 'side'], default='top')
    p.add_argument('--place-id', type=int, help='Confirmed target of rover; required for side')
    p.add_argument('--agent', default='drone-1')
    p.add_argument('--device', default='cpu')
    p.add_argument('--imgsz', type=int, default=960)
    a = p.parse_args()
    if a.view == 'side' and a.place_id is None:
        p.error('--place-id required for side view (from rover navigation)')
    import cv2
    import numpy as np
    from ultralytics import YOLO
    cfg = json.loads(Path(a.config).read_text())
    model = YOLO(a.weights)
    if dict(model.names) != CLASSES:
        p.error('Weights must contain exactly box_ok, box_damaged, foreign_object in that order')
    source = Path(a.source)
    frames = [source] if source.is_file() else sorted(
        x for x in source.rglob('*') if x.suffix.lower() in {'.jpg', '.jpeg', '.png', '.bmp'})
    if not frames:
        p.error('No images found')
    out = Path(a.attempt)
    out.mkdir(parents=True, exist_ok=True)
    log = out / 'observations.jsonl'
    detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, cfg['aruco_dictionary'])))
    for path in frames:
        frame = cv2.imread(str(path))
        if frame is None:
            raise ValueError(f'Cannot read {path}')
        h, w = frame.shape[:2]
        regions = []
        if a.view == 'side':
            # Rover must aim at one place; absence alone is not evidence of empty.
            regions = [(a.place_id, None, frame)]
        else:
            corners, ids, _ = detector.detectMarkers(frame)
            for corner, marker_id in zip(corners, ids.flatten() if ids is not None else []):
                transform = cv2.getPerspectiveTransform(
                    np.float32([[0, 0], [1, 0], [1, 1], [0, 1]]), corner[0])
                x1, y1, x2, y2 = cfg['slot_roi']
                polygon = cv2.perspectiveTransform(np.float32(
                    [[[x1, y1], [x2, y1], [x2, y2], [x1, y2]]]), transform)[0]
                if (polygon[:, 0] < 0).any() or (polygon[:, 0] >= w).any() or (
                        polygon[:, 1] < 0).any() or (polygon[:, 1] >= h).any():
                    regions.append((int(marker_id), polygon, None))
                    continue
                crop = cv2.warpPerspective(frame, cv2.getPerspectiveTransform(
                    polygon, np.float32([[0, 0], [639, 0], [639, 639], [0, 639]])), (640, 640))
                regions.append((int(marker_id), polygon, crop))
        for place_id, polygon, crop in regions:
            detections = []
            if crop is not None:
                result = model.predict(crop, conf=0.15, imgsz=a.imgsz, device=a.device, verbose=False)[0]
                for box in result.boxes:
                    detections.append({'class': CLASSES[int(box.cls.item())],
                                       'confidence': float(box.conf.item()),
                                       'xyxy': box.xyxy[0].tolist()})
            status = 'unknown' if crop is None else classify(
                detections, cfg['confidence'], cfg['allow_empty'] and a.view == 'top')
            observation = {'place_id': place_id, 'status': status, 'view': a.view,
                           'agent': a.agent, 'frame': str(path.resolve()),
                           'time': datetime.now(timezone.utc).isoformat(),
                           'roi': polygon.tolist() if polygon is not None else None,
                           'detections': detections}
            with log.open('a') as f:
                f.write(json.dumps(observation) + '\n')
    observations = [json.loads(line) for line in log.read_text().splitlines()] if log.exists() else []
    rows = summarize(observations, cfg['expected_ids'], cfg['empty_observations'])
    with (out / 'inventory.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['place_id', 'status', 'source', 'needs_rover', 'observations'])
        writer.writeheader()
        writer.writerows(rows)
    (out / 'rover_queue.json').write_text(json.dumps(
        [r['place_id'] for r in rows if r['needs_rover']], indent=2))
    print(f'{len(rows)} places; report: {out / "inventory.csv"}')


if __name__ == '__main__':
    main()
