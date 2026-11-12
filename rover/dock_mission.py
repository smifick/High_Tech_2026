"""Run a box inspection from a calibrated dock and write the rover's JSON result."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import sys
import time

import numpy as np

from .dock_navigation import Routes, dock_localization, xy
from .field import Field, MAP, read_markers
from .mission import GoalFailed, HERE, Rover, inspect, load_model
from .test_exit_marker import travel as lidar_travel

MARKERS = HERE / "aruco_map_750x450_id100.txt"
MODEL = HERE / "yolo_two_classes/rover-yolo-two-classes.pt"
FACES = {"x+": (1,0), "x-": (-1,0), "y+": (0,1), "y-": (0,-1)}


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8-sig"))


def write_json(path, value):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
                         encoding="utf-8")
    temporary.replace(path)


def vector(values, length, name):
    if (not isinstance(values, list) or len(values) != length
            or any(type(x) not in (int, float) or not math.isfinite(x) for x in values)):
        raise ValueError(f"{name} must contain {length} finite numbers")
    return np.array(values, dtype=float)


def parse_job(value, markers):
    if not isinstance(value, dict):
        raise ValueError("The job must be a JSON object")
    if type(value.get("schema_version")) is not int or value["schema_version"] != 1:
        raise ValueError("Expected schema_version: 1")
    mission_id = value.get("mission_id")
    if not isinstance(mission_id, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", mission_id):
        raise ValueError("mission_id must contain 1–64 letters, digits, underscores or hyphens")
    if not isinstance(value.get("boxes"), list):
        raise ValueError("boxes must be an array")
    boxes = {}
    for box in value["boxes"]:
        if not isinstance(box, dict):
            raise ValueError("Each box must be an object")
        marker = box.get("marker_id")
        if type(marker) is not int or marker not in markers or marker in boxes:
            raise ValueError(f"Unknown, invalid or duplicate marker_id: {marker}")
        if type(box.get("anomaly")) is not bool:
            raise ValueError(f"Marker {marker}: anomaly must be true or false")
        if "offset_m" in box:
            vector(box["offset_m"], 2, f"Marker {marker} offset_m")
        boxes[marker] = box
    return mission_id, boxes


def parse_dock(config):
    if not isinstance(config, dict):
        raise ValueError("The dock configuration must be a JSON object")
    poses = {}
    for key in ("dock_pose", "mouth_pose"):
        x, y, yaw = vector(config.get(key), 3, key + " [x, y, yaw_deg]")
        poses[key] = {"x": float(x), "y": float(y), "yaw": math.radians(yaw)}
    if np.linalg.norm(xy(poses["dock_pose"]) - xy(poses["mouth_pose"])) < .20:
        raise ValueError("Dock and mouth poses must be at least 0.20 m apart")
    if config.get("dock_mode", "nav2") not in {"nav2", "lidar"}:
        raise ValueError("dock_mode must be nav2 or lidar")
    if config.get("dock_mode") == "lidar":
        for key, start, end in (("exit_waypoints", "dock_pose", "mouth_pose"),
                                ("return_waypoints", "mouth_pose", "dock_pose")):
            items = config.get(key)
            if not isinstance(items, list) or not 1 <= len(items) <= 8:
                raise ValueError(f"{key} must contain 1–8 waypoints")
            points = [xy(poses[start]), *(vector(p, 2, key) for p in items), xy(poses[end])]
            if any(np.linalg.norm(b-a) > .8 for a,b in zip(points, points[1:])):
                raise ValueError(f"{key}: each lidar-controlled leg must be at most 0.8 m")
    return poses["dock_pose"], poses["mouth_pose"]


class Snapshot:
    def __init__(self, path):
        self.data = read_json(path)
        self.image = path.with_suffix(".png").read_bytes()
        if hashlib.sha256(self.image).hexdigest() != self.data["image_sha256"]:
            raise ValueError("Offline map PNG does not match the snapshot")

    def get(self, path, **query):
        if path == "/api/maps":
            return {"maps": [self.data["map"]]}
        if path == "/api/navigation/places" and query.get("map") == self.data["map"]["path"]:
            return self.data["places"]
        raise ValueError(f"Unavailable snapshot path: {path}")

    def read(self, path):
        if path != self.data["map"]["image_url"]:
            raise ValueError(f"Unavailable snapshot image: {path}")
        return self.image


def write_snapshot(api, path, map_path):
    metadata = next((m for m in api.get("/api/maps")["maps"] if m["path"] == map_path), None)
    if not metadata or not metadata["valid"]:
        raise ValueError(f"Map not available: {map_path}")
    image = api.read(metadata["image_url"])
    path.with_suffix(".png").write_bytes(image)
    write_json(path, {"map": metadata, "places": api.get("/api/navigation/places", map=map_path),
                      "image_sha256": hashlib.sha256(image).hexdigest()})


def plan_views(field, routes, boxes, mouth, config):
    distance = float(config.get("camera_distance_m", .72))
    if not routes.keepout + .05 <= distance <= 1.4:
        raise ValueError("camera_distance_m must exceed the box exclusion radius by at least 0.05 m")
    remaining = {marker for marker, box in boxes.items() if box["anomaly"]}
    result = {}
    start = mouth
    while remaining:
        marker = min(remaining, key=lambda i: (np.linalg.norm(routes.boxes[i] - xy(start)), i))
        remaining.remove(marker)
        views = {face: routes.view(marker, np.array(axis) @ field.rotation, mouth, distance)
                 for face, axis in FACES.items()}
        reachable = [v for v in views.values() if v is not None]
        if reachable:
            start = min(reachable, key=lambda v: np.linalg.norm(xy(v) - xy(start)))
        result[marker] = views
    return result


def make_routes(field, boxes, config):
    default = (vector(config.get("box_offset_m"), 2, "box_offset_m")
               if any("offset_m" not in box for box in boxes.values()) else None)
    centers = {marker: field.xy(np.array(field.markers[marker]) + box.get("offset_m", default))
               for marker, box in boxes.items()}
    radius = float(config.get("box_radius_m", .26))
    clearance = float(config.get("clearance_m", .23))
    if not .18 <= radius <= .70 or not .18 <= clearance <= .45:
        raise ValueError("Check box_radius_m and clearance_m against the measured geometry")
    return Routes(field, centers, radius, clearance)


def monitor_route(routes, status, target):
    if status["map_pose"] is None:
        raise RuntimeError("No position for route monitoring")
    current = xy(status["map_pose"])
    if not routes.allows(current, current):
        raise RuntimeError("Rover entered a mapped wall margin or box keep-out")
    points = status.get("planned_path", [])
    age = status.get("planned_path_age_sec")
    if (not points or age is None or age > 2 or status.get("planned_path_frame") != "map"
            or np.linalg.norm(xy(points[-1]) - xy(target)) > .18):
        return
    previous = xy(points[0])
    for point in points[1:]:
        point = xy(point)
        if not routes.allows(previous, point):
            raise RuntimeError("Nav2 planned a path through a wall margin or box keep-out")
        previous = point


def run(api, field, routes, box_views, dock, mouth, config, args, mission_id):
    lidar_dock = config.get("dock_mode") == "lidar"
    if not lidar_dock and not routes.allows(xy(dock), xy(mouth)):
        raise ValueError("Mapped dock corridor lacks clearance for the configured footprint")
    if lidar_dock and not routes.allows(xy(mouth), xy(mouth)):
        raise ValueError("The Nav2 handoff point is obstructed")
    report = {
        "schema_version": 1, "mission_id": mission_id, "status": "starting",
        "returned_to_dock": False, "navigation_stopped": None,
        "inspections": [{"marker_id": marker, "side_anomaly": None,
                         "faces": {face: {"state": "pending" if view else "unreachable"}
                                   for face, view in views.items()}}
                        for marker, views in box_views.items()],
    }
    args.output.mkdir(parents=True, exist_ok=False)
    report_path = args.output / "result.json"
    write_json(report_path, report)
    if not box_views:
        report.update(status="no_anomalous_boxes", returned_to_dock=None)
        write_json(report_path, report)
        return report
    if not any(view for views in box_views.values() for view in views.values()):
        report.update(status="partial", returned_to_dock=None)
        write_json(report_path, report)
        return report
    owned = False
    starting = False
    current = None
    error = None
    try:
        state, _ = api.telemetry()
        if state["running"] or any(state["external"].values()) or not state["prerequisites"]["ready"]:
            raise RuntimeError("Nav2 must be idle with all navigation topics available")
        api.camera()
        model = load_model(args.weights)
        model.predict(np.zeros((512, 512, 3), dtype=np.uint8), imgsz=640,
                      device=args.device, verbose=False)
        radius = float(config.get("dock_search_radius_m", .55))
        if not .20 <= radius <= .90:
            raise ValueError("dock_search_radius_m must be between .20 and .90")
        lidar_offset = vector(config.get("lidar_offset_m", [.0662, 0.0]), 2, "lidar_offset_m")
        initial, quality = dock_localization(api, field, dock, radius, lidar_offset)
        state, odom = api.telemetry()
        if math.hypot(odom["vx"], odom["vy"]) > .03 or abs(odom["wz"]) > .08:
            raise RuntimeError("Rover must be stationary in the dock")
        deadline = time.monotonic() + args.seconds
        inspection_deadline = deadline - args.return_reserve
        report.update(status="leaving_dock", initial_pose=initial, lidar_localization=quality)
        write_json(report_path, report)

        def dock_leg(current_pose, points, until):
            for point in points:
                actual = lidar_travel(api, current_pose, point, .045, deadline=until)
                seed = {"x": float(actual[0]), "y": float(actual[1]), "yaw": current_pose["yaw"]}
                current_pose, _ = dock_localization(api, field, seed, .20, lidar_offset)
                if np.linalg.norm(xy(current_pose) - point) > .12:
                    raise RuntimeError("Lidar did not confirm the dock waypoint")
            return current_pose

        if lidar_dock:
            if np.linalg.norm(xy(initial) - xy(dock)) > .12:
                raise RuntimeError("Rover is not at the calibrated parking position")
            exit_points = [vector(p, 2, "exit_waypoints") for p in config["exit_waypoints"]]
            initial = dock_leg(initial, [*exit_points, xy(mouth)], inspection_deadline)
            _, odom = api.telemetry()
            if not routes.allows(xy(initial), xy(mouth)):
                raise RuntimeError("Lidar-localized exit is outside the Nav2 handoff area")
        elif not routes.allows(xy(initial), xy(mouth)):
            raise RuntimeError("Lidar-localized start is outside the safe exit corridor")
        reference = initial, odom
        report.update(status="inspecting", nav_initial_pose=initial)
        write_json(report_path, report)

        def travel(target, until, first=False):
            nonlocal owned, starting, current
            waypoints = [target] if first else routes.waypoints(current, target)
            if waypoints is None:
                raise GoalFailed("No path with mapped wall and box clearance")
            for waypoint in waypoints:
                if time.monotonic() + 2 >= until:
                    raise TimeoutError("Insufficient time to start another goal")
                if first:
                    starting = True
                    request = api.post("start", map=field.info["path"], initial_pose=initial, goal=waypoint)
                    owned = True
                    starting = False
                else:
                    request = api.post("goal", goal=waypoint)
                try:
                    api.wait(request, waypoint, min(until, time.monotonic() + 75),
                             field.info["path"], reference=reference,
                             monitor=lambda status: monitor_route(routes, status, waypoint))
                except GoalFailed:
                    actual = api.status()["map_pose"]
                    if actual is None or not routes.allows(xy(actual), xy(actual)):
                        raise RuntimeError("Cannot verify rover pose after a rejected goal")
                    current = actual
                    raise
                current = api.status()["map_pose"]
                if current is None:
                    raise RuntimeError("No map pose after a goal")

        travel(mouth, inspection_deadline, first=True)
        for result in report["inspections"]:
            marker = result["marker_id"]
            for face, view in box_views[marker].items():
                if view is None or time.monotonic() + 20 >= inspection_deadline:
                    continue
                face_result = result["faces"][face]
                try:
                    travel(view, inspection_deadline)
                except GoalFailed as failure:
                    face_result.update(state="unreachable", reason=str(failure))
                    continue
                try:
                    vote, confidence, reason = inspect(
                        api, model, args, f"{marker}_{face}", args.output, inspection_deadline)
                    face_result.update(state={"occupied": "normal", "anomaly": "anomaly"}.get(vote, "unknown"),
                                       confidence=confidence or None, reason=reason)
                except (OSError, RuntimeError, ValueError) as failure:
                    face_result.update(state="unknown", reason=str(failure))
                finally:
                    states = {value["state"] for value in result["faces"].values()}
                    result["side_anomaly"] = True if "anomaly" in states else False if states == {"normal"} else None
                    write_json(report_path, report)
        report["status"] = "returning"
        write_json(report_path, report)
        travel(mouth, deadline)
        if lidar_dock:
            api.stop()
            owned = False
            report["navigation_stopped"] = True
            report["status"] = "parking"
            write_json(report_path, report)
            current, _ = dock_localization(api, field, current, .20, lidar_offset)
            ingress = [vector(p, 2, "return_waypoints") for p in config["return_waypoints"]]
            current = dock_leg(current, [*ingress, xy(dock)], deadline)
        else:
            travel(dock, deadline)
        report.update(status="completed" if all(item["side_anomaly"] is not None
                                                for item in report["inspections"]) else "partial",
                      returned_to_dock=True, final_pose=current)
    except (Exception, KeyboardInterrupt) as failure:
        report.update(status="interrupted", error=str(failure) or type(failure).__name__)
        error = failure
    finally:
        try:
            if starting:
                state = api.status()
                owned = (state["running"] and state["map"] == field.info["path"]
                         and state["initial_pose"] == initial and state["goal"] == mouth)
            if owned:
                api.stop()
                report["navigation_stopped"] = True
        except (OSError, RuntimeError) as failure:
            report.update(status="interrupted", navigation_stopped=False, stop_error=str(failure))
            error = error or failure
        finally:
            for item in report["inspections"]:
                for face in item["faces"].values():
                    if face["state"] == "pending":
                        face["state"] = "not_inspected"
            write_json(report_path, report)
    if error is not None:
        raise error
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["plan", "run", "snapshot"])
    parser.add_argument("--url", default="http://10.11.3.195:8765")
    parser.add_argument("--map", default=MAP)
    parser.add_argument("--job", type=Path)
    parser.add_argument("--dock", type=Path)
    parser.add_argument("--snapshot", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--weights", type=Path, default=MODEL)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--confidence", type=float, default=.60)
    parser.add_argument("--seconds", type=int, default=540)
    parser.add_argument("--return-reserve", type=int, default=100)
    parser.set_defaults(empty_verified=False)
    args = parser.parse_args()
    api = Rover(args.url)
    if args.mode == "snapshot":
        write_snapshot(api, args.output, args.map)
        return
    if not args.job or not args.dock or args.mode == "run" and args.snapshot:
        parser.error("plan/run require --job and --dock; --snapshot is only supported for plan")
    if not .20 <= args.confidence <= 1 or not 30 <= args.return_reserve < args.seconds <= 600:
        parser.error("Check confidence, mission time and return reserve")
    markers = read_markers(MARKERS)
    mission_id, boxes = parse_job(read_json(args.job), markers)
    config = read_json(args.dock)
    dock, mouth = parse_dock(config)
    if args.mode == "run" and config.get("simulation_only", False):
        raise ValueError("This dock configuration is for offline route testing")
    field = Field(Snapshot(args.snapshot) if args.snapshot else api, markers, args.map)
    routes = make_routes(field, boxes, config)
    if config.get("dock_mode") != "lidar" and not routes.allows(xy(dock), xy(mouth)):
        raise ValueError("The mapped exit between dock and mouth lacks clearance")
    if not routes.allows(xy(mouth), xy(mouth)):
        raise ValueError("The Nav2 handoff point is obstructed")
    box_views = plan_views(field, routes, boxes, mouth, config)
    if args.mode == "plan":
        write_json(args.output, {"mission_id": mission_id, "dock": dock, "mouth": mouth,
                                 "box_centers": {i: center.tolist() for i, center in routes.boxes.items()},
                                 "inspection_goals": box_views})
        field.preview(args.output.with_suffix(".png"), dock,
                      [mouth, *(v for views in box_views.values() for v in views.values() if v)])
        return
    result = run(api, field, routes, box_views, dock, mouth, config, args, mission_id)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    try:
        main()
    except (ValueError, KeyError, TypeError, OSError, RuntimeError) as failure:
        sys.exit(str(failure))
