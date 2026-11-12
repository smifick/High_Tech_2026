"""Низкоскоростная проверка выезда из ниши и подхода к метке 100."""
import argparse
import json
import math
import time

import numpy as np

from .dock_navigation import dock_localization, xy
from .field import Field, MAP, read_markers
from .mission import HERE, Rover

DOCK_HINT = {"x": 5.973, "y": 4.861, "yaw": math.radians(25)}
LIDAR_OFFSET = np.array([.0662, 0.])
SPEED = .08
APPROACH = np.array([6.15, 5.45])


def rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, s], [-s, c]])


def check_idle(rover):
    nav, odom = rover.telemetry()
    if nav["running"] or any(nav["external"].values()) or rover.get("/api/drive")["active"]:
        raise RuntimeError("Другое управление ровером уже активно")
    if math.hypot(odom["vx"], odom["vy"]) > .03 or abs(odom["wz"]) > .08:
        raise RuntimeError("Ровер должен стоять перед началом участка")
    return odom


def check_scan(rover, forward, remaining):
    scan = rover.get("/api/lidar/status", topic="/scan_filtered")
    if (scan["frame_id"] != "lidar_link" or scan["age_sec"] is None
            or scan["age_sec"] > .35 or scan["valid_points"] < 150):
        raise RuntimeError("Нет свежего скана лидара")
    points = -np.asarray(scan["points"]) + LIDAR_OFFSET
    along = points @ forward
    cross = points @ [-forward[1], forward[0]]
    if np.any((along > -.17) & (along < min(.50, remaining + .20)) & (abs(cross) < .19)):
        raise RuntimeError("Лидар видит препятствие в коридоре")


def travel(rover, start, target, tolerance, *, deadline=None):
    odom = check_idle(rover)
    transform = rotation(start["yaw"] - odom["yaw"])
    position = xy(start)
    end = time.monotonic() + 55
    if deadline is not None:
        end = min(end, deadline)
    while time.monotonic() < end:
        nav, current = rover.telemetry()
        if nav["running"] or any(nav["external"].values()):
            raise RuntimeError("Запустился другой контроллер")
        position = xy(start) + (xy(current) - xy(odom)) @ transform
        delta = target - position
        remaining = float(np.linalg.norm(delta))
        if remaining <= tolerance:
            return position
        heading_change = math.remainder(current["yaw"] - odom["yaw"], 2 * math.pi)
        if abs(heading_change) > .12:
            raise RuntimeError("Курс изменился больше допустимого")
        forward = delta / remaining @ rotation(-start["yaw"] - heading_change)
        pulse_end = min(end, time.monotonic() + min(1.0, remaining / SPEED))
        try:
            while time.monotonic() < pulse_end:
                check_scan(rover, forward, remaining)
                command = {"linear_x": SPEED * float(forward[0]),
                           "linear_y": SPEED * float(forward[1]), "angular_z": 0}
                if not json.loads(rover.read("/api/drive/command", command))["ok"]:
                    raise RuntimeError("Ровер отклонил команду движения")
                time.sleep(.07)
        finally:
            rover.read("/api/drive/stop", {})
        time.sleep(.4)
    raise TimeoutError(f"Не достигли цели за отведённое время: {position.round(3)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://10.11.3.195:8765")
    parser.add_argument("--move", action="store_true", help="Запустить движение с проверенной стартовой позы")
    args = parser.parse_args()
    rover = Rover(args.url)
    field = Field(rover, read_markers(HERE / "aruco_map_750x450_id100.txt"), MAP)
    check_idle(rover)
    start, quality = dock_localization(rover, field, DOCK_HINT, .30, LIDAR_OFFSET)
    if np.linalg.norm(xy(start) - xy(DOCK_HINT)) > .12:
        raise RuntimeError(f"Ровер не в исходной точке ниши: {start}")
    forward = np.array([math.cos(start["yaw"]), math.sin(start["yaw"])])
    exit_target = xy(start) + .30 * forward
    marker = field.xy(field.markers[100])
    check_scan(rover, np.array([1., 0.]), .30)
    print(f"Старт {start}; совпадение лидара {quality}; метка 100 {marker}", flush=True)
    if not args.move:
        return
    reached = travel(rover, start, exit_target, .025)
    print(f"Выезд: {np.linalg.norm(reached - xy(start)):.2f} м по одометрии", flush=True)
    estimate = dict(x=float(reached[0]), y=float(reached[1]), yaw=start["yaw"])
    localized, _ = dock_localization(rover, field, estimate, .24, LIDAR_OFFSET)
    reached = travel(rover, localized, APPROACH, .04)
    estimate = dict(x=float(reached[0]), y=float(reached[1]), yaw=localized["yaw"])
    localized, _ = dock_localization(rover, field, estimate, .20, LIDAR_OFFSET)
    final = travel(rover, localized, marker, .04)
    print(f"До центра метки 100 по расчёту: {np.linalg.norm(final - marker):.2f} м", flush=True)


if __name__ == "__main__":
    main()
