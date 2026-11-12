"""Ground inspection for case 1. Run from the team checkout on Windows."""

import argparse
import csv
import json
import math
from pathlib import Path
import sys
import time
from urllib.error import HTTPError
from urllib.parse import urlencode
from urllib.request import Request, urlopen

import cv2
import numpy as np

from .field import Field, MAP, read_markers

HERE = Path(__file__).resolve().parent
WEIGHTS = (HERE.parent / "YOLO/YOLO11n_models/YOLO_model_classificator"
           / "weights/best_for_classificator.pt")
STATES = {"occupied", "empty", "anomaly", "unknown"}
LABELS = {"occupied": "занято", "empty": "пусто", "anomaly": "аномальное",
          "unknown": "не определено"}
PENDING = {"queued", "sending", "active"}


class GoalFailed(RuntimeError):
    """The server confirmed a terminal failure of the requested goal."""


def check_localization(initial_pose, initial_odom, odom, map_pose):
    """Compare AMCL with the short-range odometry prediction from the start."""
    rotation = initial_pose["yaw"] - initial_odom["yaw"]
    dx = odom["x"] - initial_odom["x"]
    dy = odom["y"] - initial_odom["y"]
    x = initial_pose["x"] + math.cos(rotation) * dx - math.sin(rotation) * dy
    y = initial_pose["y"] + math.sin(rotation) * dx + math.cos(rotation) * dy
    position_error = math.hypot(map_pose["x"] - x, map_pose["y"] - y)
    yaw_error = abs(math.remainder(map_pose["yaw"] - odom["yaw"] - rotation, 2 * math.pi))
    # Wheel slip can accumulate during a mission. Allow drift with travel distance.
    if position_error > min(0.55, 0.30 + 0.06 * math.hypot(dx, dy)) or yaw_error > 0.40:
        raise RuntimeError(
            f"Localization disagrees with odometry ({position_error:.2f} m, "
            f"{math.degrees(yaw_error):.0f} deg)"
        )


class Rover:
    def __init__(self, url):
        self.url = url.rstrip("/")

    def read(self, path, data=None, **query):
        url = self.url + path
        if query:
            url += "?" + urlencode(query)
        body = None if data is None else json.dumps(data, allow_nan=False).encode()
        try:
            with urlopen(Request(url, body, {"Content-Type": "application/json"}), timeout=4) as response:
                return response.read()
        except HTTPError as error:
            detail = error.read(4096).decode(errors="replace")
            try:
                detail = json.loads(detail).get("error", detail)
            except (ValueError, AttributeError):
                detail = error.reason
            raise RuntimeError(f"{path}: HTTP {error.code}: {detail}") from error

    def get(self, path, **query):
        return json.loads(self.read(path, **query))

    def post(self, operation, **data):
        result = json.loads(self.read("/api/navigation/" + operation, data))
        if not result["ok"]:
            raise RuntimeError(result.get("error", operation))
        return result["navigation"]

    def status(self):
        return self.get("/api/navigation/status")

    def stop(self):
        self.post("stop")
        deadline = time.monotonic() + 14
        while time.monotonic() < deadline:
            if not self.status()["running"]:
                return
            time.sleep(0.25)
        raise TimeoutError("The rover did not confirm that navigation stopped")

    def camera(self):
        status = self.get("/api/camera/media/status", mode="live")
        if not status["frame_ready"] or status["age_sec"] > 1:
            raise RuntimeError("No fresh live image")
        return status

    def frame(self):
        status = self.camera()
        return self.read("/api/camera/media/frame", mode="live", after=status["sequence"])

    def telemetry(self):
        report = self.get("/api/status")
        age = report["topics"]["odom"]["age_sec"]
        if not report["odom"] or age is None or age > 0.5:
            raise RuntimeError("Fresh wheel/IMU odometry is unavailable")
        return report["navigation"], report["odom"]

    def wait(self, request, goal, deadline, map_path, reference=None, monitor=None):
        generation = request["generation"]
        while time.monotonic() < deadline:
            if reference is None:
                status = self.status()
            else:
                status, odom = self.telemetry()
            if (not status["running"] or status["map"] != map_path
                    or status["generation"] != generation):
                raise RuntimeError("Navigation session changed")
            state = status["goal_state"]
            if reference is not None and state in {"sending", "active", "succeeded"}:
                if status["map_pose"] is None:
                    raise RuntimeError("Map localization unavailable during navigation")
                check_localization(*reference, odom, status["map_pose"])
            if monitor is not None and state in {"sending", "active", "succeeded"}:
                monitor(status)
            if state == "succeeded":
                pose = status["map_pose"]
                if pose is None or math.hypot(pose["x"] - goal["x"], pose["y"] - goal["y"]) > 0.16:
                    raise RuntimeError("Map pose does not match reached goal")
                error = math.remainder(pose["yaw"] - goal["yaw"], 2 * math.pi)
                if abs(error) > 0.15:
                    raise RuntimeError("Camera heading does not match reached goal")
                return
            if state not in PENDING:
                if state in {"aborted", "rejected"} and status["phase"] == "running":
                    raise GoalFailed(f"Nav2 {state}: {status.get('goal_message')}")
                raise RuntimeError(f"Nav2 {state}: {status.get('goal_message')}")
            time.sleep(0.2)
        raise TimeoutError("Navigation deadline expired")


def plan(field, queue, pose, box_offset):
    goals = {}
    xy = np.array([pose["x"], pose["y"]])
    remaining = set(queue)
    while remaining:
        options = [(marker, field.approach(marker, box_offset, xy))
                   for marker in sorted(remaining)]
        options = [(marker, goal) for marker, goal in options if goal is not None]
        if not options:
            break
        marker, goal = min(options, key=lambda entry: math.hypot(
            entry[1]["x"] - xy[0], entry[1]["y"] - xy[1]))
        goals[marker] = goal
        xy = np.array([goal["x"], goal["y"]])
        remaining.remove(marker)
    return goals


def load_input(markers, observations, ids):
    rows = {i: {"marker_id": i, "state": "unknown", "source": "",
                "rover_state": "", "reason": "", "confidence": ""} for i in sorted(markers)}
    states = {}
    if observations:
        for line in observations.read_text(encoding="utf-8-sig").splitlines():
            if not line.strip():
                continue
            observation = json.loads(line)
            marker = int(observation["marker_id"])
            state = observation["state"]
            if marker not in rows or state not in STATES:
                raise ValueError(f"Invalid observation: {observation}")
            states.setdefault(marker, set()).add(state)
    queue = set(ids)
    for marker, values in states.items():
        state = next(iter(values)) if len(values) == 1 else "unknown"
        rows[marker].update(state=state, source="drone")
        if state in {"anomaly", "unknown"}:
            queue.add(marker)
    for marker in queue:
        rows[marker].update(state="unknown", source="", rover_state="unknown", reason="pending")
    return rows, queue


def save_inventory(path, rows):
    temporary = path.with_suffix(".tmp")
    with temporary.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=[*next(iter(rows.values())), "status"], delimiter=";")
        writer.writeheader()
        writer.writerows({**row, "status": LABELS[row["state"]]} for row in rows.values())
    temporary.replace(path)


def load_model(weights):
    from ultralytics import YOLO

    if not weights.is_file():
        raise FileNotFoundError(weights)
    model = YOLO(str(weights))
    expected = {0: "box_hitech", 1: "box_hitech_defect"}
    if model.task != "detect" or model.names != expected:
        raise ValueError(f"Unexpected model classes: {model.names}")
    return model


def infer(model, frame, confidence, device):
    image = cv2.imdecode(np.frombuffer(frame, np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("Camera returned an invalid JPEG")
    height, width = image.shape[:2]
    result = model.predict(image, conf=0.20, imgsz=640, verbose=False, device=device)[0]
    candidates = []
    for x1, y1, x2, y2, score, label in result.boxes.data.cpu().tolist():
        if 0.34 * width <= (x1 + x2) / 2 <= 0.66 * width and 0.4 * height <= y2 <= 0.98 * height:
            candidates.append((int(label), score))
    if not candidates:
        return "no_detection", 0.0
    if len(candidates) != 1 or candidates[0][1] < confidence:
        return "unknown", max(score for _, score in candidates)
    label, score = candidates[0]
    return ("anomaly" if label == 1 else "occupied"), score


def inspect(rover, model, args, marker, directory, deadline):
    time.sleep(0.4)
    votes = []
    for index in range(3):
        if time.monotonic() >= deadline:
            raise TimeoutError("Attempt deadline expired")
        frame = rover.frame()
        (directory / f"{marker}_{index + 1}.jpg").write_bytes(frame)
        votes.append(infer(model, frame, args.confidence, args.device))
        time.sleep(0.2)
    states = {state for state, _ in votes}
    if len(states) == 1 and states <= {"anomaly", "occupied"}:
        return next(iter(states)), round(min(score for _, score in votes), 3), "consistent_camera_frames"
    if states == {"no_detection"} and args.empty_verified:
        return "empty", "", "calibrated_view_without_detection"
    return "unknown", "", "no_detection_or_inconsistent_frames"


def execute(rover, args, field, pose, goals, rows):
    status = rover.status()
    if (status["running"] or any(status["external"].values())
            or not status["prerequisites"]["ready"]):
        raise RuntimeError("Expected an idle rover with navigation topics available")
    rover.camera()
    model = load_model(args.weights)
    # Run the model before enabling motion, including device and checkpoint checks.
    infer(model, rover.frame(), args.confidence, args.device)
    args.output.mkdir(parents=True, exist_ok=False)
    field.preview(args.output / "route.png", pose, goals.values())
    for marker, row in rows.items():
        if row["reason"] == "pending" and marker not in goals:
            row["reason"] = "no_traversable_viewpoint"
    report = args.output / "inventory.csv"
    save_inventory(report, rows)
    deadline = time.monotonic() + args.seconds
    owned = False
    starting = False
    reference = None
    try:
        for marker, goal in goals.items():
            if time.monotonic() + 4 >= deadline:
                break
            print(f"{marker}: navigating to {goal}", flush=True)
            try:
                if not owned:
                    _, odom = rover.telemetry()
                    reference = pose, odom
                    starting = True
                    request = rover.post("start", map=args.map, initial_pose=pose, goal=goal)
                    owned = True
                    starting = False
                else:
                    request = rover.post("goal", goal=goal)
                rover.wait(request, goal, min(deadline - 3, time.monotonic() + 75), args.map,
                           reference=reference)
                state, confidence, reason = inspect(rover, model, args, marker, args.output, deadline - 3)
                rows[marker].update(state=state, source="rover", rover_state=state,
                                    reason=reason, confidence=confidence)
                print(f"{marker}: {LABELS[state]}", flush=True)
            except GoalFailed as exc:
                rows[marker]["reason"] = str(exc)
                print(f"{marker}: {exc}", file=sys.stderr, flush=True)
            except Exception as exc:
                rows[marker]["reason"] = str(exc)
                raise
            finally:
                save_inventory(report, rows)
    finally:
        try:
            if starting:
                # A timed-out POST may already have started the requested process.
                status = rover.status()
                owned = (status["running"] and status["map"] == args.map
                         and status["initial_pose"] == pose and status["goal"] == goal)
            if owned:
                rover.stop()
        finally:
            for row in rows.values():
                if row["reason"] == "pending":
                    row["reason"] = "attempt_ended_before_inspection"
            save_inventory(report, rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["check", "plan", "run", "stop"])
    parser.add_argument("--url", default="http://10.11.3.195:8765")
    parser.add_argument("--map", default=MAP)
    parser.add_argument("--markers", type=Path, default=HERE / "aruco_map_750x450_id100.txt")
    parser.add_argument("--weights", type=Path, default=WEIGHTS)
    parser.add_argument("--observations", type=Path)
    parser.add_argument("--ids", nargs="+", type=int, default=[])
    start = parser.add_mutually_exclusive_group()
    start.add_argument("--start", nargs=3, type=float, metavar=("X", "Y", "YAW_DEG"))
    start.add_argument("--start-px", nargs=3, type=float, metavar=("COL", "ROW", "YAW_DEG"))
    parser.add_argument("--box-offset", nargs=2, type=float, metavar=("DX", "DY"))
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--confidence", type=float, default=0.60)
    parser.add_argument("--seconds", type=int, default=540)
    parser.add_argument("--empty-verified", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    rover = Rover(args.url)
    if args.mode == "stop":
        rover.stop()
        print("Navigation stopped")
        return
    markers = read_markers(args.markers)
    if not set(args.ids) <= markers.keys():
        parser.error("An inspection ID is missing from the ArUco map")
    field = Field(rover, markers, args.map)
    if args.mode == "check":
        print(json.dumps({"map": field.info["path"], "revision": field.info["revision"],
                          "anchors": field.places["places"], "residual_m": field.residual.tolist(),
                          "rotation_deg": math.degrees(math.atan2(field.rotation[0, 1],
                                                                   field.rotation[0, 0])),
                          "translation_m": field.offset.tolist(),
                          "marker_108_pixel": field.pixel(field.xy(markers[108])),
                          "camera": rover.camera(), "navigation": rover.status()["prerequisites"]},
                         indent=2, ensure_ascii=False))
        if args.output:
            field.preview(args.output)
        return
    if not (args.start or args.start_px) or args.box_offset is None or args.output is None:
        parser.error("Plan and run require a start pose, box offset and output path")
    if not all(map(math.isfinite, [*(args.start or args.start_px), *args.box_offset])):
        parser.error("Pose and offset must be finite")
    if not 0.20 <= args.confidence <= 1 or not 10 <= args.seconds <= 600:
        parser.error("Confidence must be in [0.20, 1] and seconds in [10, 600]")
    x, y, yaw = args.start or args.start_px
    if args.start_px:
        x, y = map(float, field.world((x, y)))
    pose = dict(x=x, y=y, yaw=math.radians(yaw))
    rows, queue = load_input(markers, args.observations, args.ids)
    if not queue:
        parser.error("Supply inspection IDs or observations with disputed places")
    goals = plan(field, queue, pose, np.asarray(args.box_offset))
    if args.mode == "plan":
        field.preview(args.output, pose, goals.values())
        print(json.dumps({"start": pose, "goals": goals, "unreachable": sorted(queue - goals.keys())},
                         indent=2))
    else:
        execute(rover, args, field, pose, goals, rows)


if __name__ == "__main__":
    try:
        main()
    except (ValueError, KeyError, OSError, RuntimeError) as error:
        sys.exit(str(error))
