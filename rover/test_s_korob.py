"""Выезд на 50 см и осмотр коробок в трёх направлениях."""
import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np

from .dock_mission import MODEL, write_json
from .mission import load_model
from .test_exit_marker import (DOCK_HINT, Field, HERE, LIDAR_OFFSET, MAP, Rover,
                               check_idle, dock_localization, read_markers, travel, xy)


def pair_boxes(boxes, markers):
    proposals = []
    for left, top, right, bottom, score, label in boxes:
        width = right - left
        height = bottom - top
        candidates = []
        for marker, (x, y) in markers.items():
            dx = abs(x - (left + right) / 2)
            dy = y - bottom
            if dx <= .9 * width and -.2 * height <= dy <= .7 * height:
                candidates.append((math.hypot(dx / width, dy / height), marker))
        candidates.sort()
        marker = candidates[0][1] if candidates and (len(candidates) == 1
                    or candidates[1][0] - candidates[0][0] > .20) else None
        proposals.append({"marker_id": marker, "anomaly": bool(int(label)),
                          "confidence": float(score),
                          "box_xyxy": [round(v) for v in (left, top, right, bottom)]})
    counts = Counter(item["marker_id"] for item in proposals)
    for item in proposals:
        if counts[item["marker_id"]] > 1:
            item["marker_id"] = None
    return proposals


def turn(rover, target_yaw):
    check_idle(rover)
    deadline = time.monotonic() + 15
    try:
        while time.monotonic() < deadline:
            nav, odom = rover.telemetry()
            if nav["running"] or any(nav["external"].values()):
                raise RuntimeError("Запустился другой контроллер")
            error = math.remainder(target_yaw - odom["yaw"], 2 * math.pi)
            if abs(error) < math.radians(4):
                return
            laser = rover.get("/api/lidar/status", topic="/scan_filtered")
            if (laser["frame_id"] != "lidar_link" or laser["age_sec"] is None
                    or laser["age_sec"] > .35 or laser["valid_points"] < 150):
                raise RuntimeError("Нет свежего скана перед поворотом")
            points = -np.asarray(laser["points"]) + LIDAR_OFFSET
            if np.any(np.linalg.norm(points, axis=1) < .23):
                raise RuntimeError("Недостаточно места для поворота")
            speed = math.copysign(min(.22, max(.12, abs(error))), error)
            if not json.loads(rover.read("/api/drive/command", {"angular_z": speed}))["ok"]:
                raise RuntimeError("Ровер отклонил команду поворота")
            time.sleep(.055)
        raise TimeoutError("Ровер не достиг заданного курса")
    finally:
        rover.read("/api/drive/stop", {})


def scan(rover, model, detector, marker_ids, output, device, sweep=False):
    votes = defaultdict(list)
    frames = []
    _, odom = rover.telemetry()
    heading = odom["yaw"]
    angles = (0, 45, -45, 0) if sweep else (0,)
    for angle in angles:
        if sweep and (angle or frames):
            turn(rover, heading + math.radians(angle))
            time.sleep(.45)
        # The final zero restores the original heading.
        if sweep and angle == 0 and frames:
            break
        for _ in range(3 if sweep else 5):
            frames.append(capture(rover, model, detector, marker_ids, output, device,
                                  len(frames) + 1, angle, votes))
    results = []
    for marker, observations in sorted(votes.items()):
        strong = [o for o in observations if o["confidence"] >= .60]
        labels = {o["anomaly"] for o in strong}
        verdict = next(iter(labels)) if len(strong) >= 3 and len(labels) == 1 else None
        results.append({"marker_id": marker, "anomaly": verdict,
                        "matched_frames": len(observations), "confident_frames": len(strong)})
    return {"boxes": results, "frames": frames,
            "unassigned_detections": sum(o["marker_id"] is None for f in frames for o in f["detections"])}


def capture(rover, model, detector, marker_ids, output, device, index, angle, votes):
    raw = rover.frame()
    image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("Камера вернула повреждённый кадр")
    corners, ids, _ = detector.detectMarkers(image)
    markers = {} if ids is None else {
        int(marker): quad.reshape(4, 2).mean(axis=0)
        for marker, quad in zip(ids.ravel(), corners) if int(marker) in marker_ids}
    prediction = model.predict(image, conf=.20, imgsz=640, device=device, verbose=False)[0]
    observations = pair_boxes(prediction.boxes.data.cpu().tolist(), markers)
    for box in observations:
        if box["marker_id"] is not None:
            votes[box["marker_id"]].append(box)
    file = f"frame_{index:02d}.jpg"
    (output / file).write_bytes(raw)
    time.sleep(.25)
    return {"file": file, "heading_offset_deg": angle,
            "visible_marker_ids": sorted(markers), "detections": observations}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://10.11.3.195:8765")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--weights", type=Path, default=MODEL)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--dictionary", choices=("DICT_4X4_250", "DICT_4X4_1000"),
                        default="DICT_4X4_1000")
    parser.add_argument("--move", action="store_true")
    parser.add_argument("--sweep", action="store_true",
                        help="Осмотреться с поворотами с текущей позиции, без выезда")
    args = parser.parse_args()
    rover = Rover(args.url)
    markers = read_markers(HERE / "aruco_map_750x450_id100.txt")
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, args.dictionary)))
    check_idle(rover)
    model = load_model(args.weights)
    model.predict(np.zeros((640, 640, 3), np.uint8), device=args.device, verbose=False)
    rover.camera()
    args.output.mkdir(parents=True, exist_ok=False)
    report = {"status": "started", "drive_m": None, "dictionary": args.dictionary}
    try:
        if args.move:
            field = Field(rover, markers, MAP)
            start, _ = dock_localization(rover, field, DOCK_HINT, .25, LIDAR_OFFSET)
            if np.linalg.norm(xy(start) - xy(DOCK_HINT)) > .12:
                raise RuntimeError(f"Ровер вне стартовой позиции: {start}")
            direction = np.array([math.cos(start["yaw"]), math.sin(start["yaw"])])
            final = travel(rover, start, xy(start) + .50 * direction, .025)
            report["drive_m"] = round(float(np.linalg.norm(final - xy(start))), 3)
            time.sleep(.4)
        check_idle(rover)
        report.update(scan(rover, model, detector, markers, args.output, args.device,
                           sweep=args.move or args.sweep))
        report["status"] = "completed"
    except (Exception, KeyboardInterrupt) as error:
        report.update(status="interrupted", error=str(error) or type(error).__name__)
        raise
    finally:
        write_json(args.output / "result.json", report)
        print(json.dumps({k: v for k, v in report.items() if k != "frames"},
                         ensure_ascii=False, indent=2), flush=True)


if __name__ == "__main__":
    main()
