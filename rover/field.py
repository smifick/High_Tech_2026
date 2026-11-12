"""Registration of ArUco coordinates against the rover's saved map."""

import math
from pathlib import Path

import cv2
import numpy as np

MAP = "archive/field_01_20261111_125402/map.yaml"
REVISION = "1794390472624986073-1794390472624986073"
ANCHORS = (100, 108, 136, 144)


def read_markers(path):
    markers = {}
    for line in Path(path).read_text(encoding="utf-8-sig").splitlines():
        fields = line.split("#", 1)[0].split()
        if not fields:
            continue
        marker = int(fields[0])
        xy = tuple(map(float, fields[2:4]))
        if len(fields) != 8 or marker in markers or not all(map(math.isfinite, xy)):
            raise ValueError(f"Invalid ArUco row: {line}")
        markers[marker] = xy
    if not markers:
        raise ValueError("ArUco map is empty")
    return markers


def register(markers, places):
    points = {int(p["name"]): (p["x"], p["y"]) for p in places if p["name"].isdigit()}
    src = np.array([markers[i] for i in ANCHORS])
    dst = np.array([points[i] for i in ANCHORS])
    a, b = src - src.mean(axis=0), dst - dst.mean(axis=0)
    u, _, vt = np.linalg.svd(a.T @ b)
    rotation = u @ vt
    if np.linalg.det(rotation) < 0:
        raise ValueError("Saved control points imply a mirrored field")
    offset = dst.mean(axis=0) - src.mean(axis=0) @ rotation
    residual = np.linalg.norm(src @ rotation + offset - dst, axis=1)
    if max(residual) > 0.15:
        raise ValueError(f"Control points disagree with ArUco map: {residual.tolist()}")
    return rotation, offset, residual


class Field:
    def __init__(self, api, markers, map_path):
        self.info = next((m for m in api.get("/api/maps")["maps"] if m["path"] == map_path), None)
        if not self.info or not self.info["valid"]:
            raise ValueError(f"Saved map unavailable: {map_path}")
        if map_path == MAP and self.info["revision"] != REVISION:
            raise ValueError("Saved occupancy map revision changed")
        self.places = api.get("/api/navigation/places", map=map_path)
        self.rotation, self.offset, self.residual = register(markers, self.places["places"])
        self.markers = markers
        self.image = cv2.imdecode(np.frombuffer(api.read(self.info["image_url"]), np.uint8),
                                  cv2.IMREAD_GRAYSCALE)
        self.resolution = self.info["resolution"]
        self.origin = np.array(self.info["origin"][:2])
        if self.image is None or self.image.shape != (self.info["height_px"], self.info["width_px"]):
            raise ValueError("Saved map image dimensions differ from metadata")
        if abs(self.info["origin"][2]) > 1e-6:
            raise ValueError("This map must have an axis-aligned occupancy grid")
        free = self.image if not self.info["negate"] else 255 - self.image
        free = (free > 255 * (1 - self.info["free_thresh"])).astype(np.uint8)
        padded = np.pad(free, 1)
        clearance = cv2.distanceTransform(padded, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
        self.clearance = clearance[1:-1, 1:-1] * self.resolution
        _, self.components = cv2.connectedComponents((self.clearance >= 0.28).astype(np.uint8),
                                                       connectivity=4)
        self.approaches = {}

    def xy(self, field_xy):
        return np.array(field_xy) @ self.rotation + self.offset

    def pixel(self, xy):
        x, y = np.floor((np.asarray(xy) - self.origin) / self.resolution).astype(int)
        return int(x), int(self.image.shape[0] - 1 - y)

    def world(self, pixel):
        x, y = pixel
        return self.origin + self.resolution * np.array([x + 0.5, self.image.shape[0] - y - 0.5])

    def cell(self, grid, xy):
        x, y = self.pixel(xy)
        if 0 <= y < grid.shape[0] and 0 <= x < grid.shape[1]:
            return grid[y, x]
        return 0

    def approach(self, marker, box_offset, start):
        component = self.cell(self.components, start)
        if component == 0:
            raise ValueError("Initial pose is outside the traversable part of the map")
        key = (marker, tuple(box_offset), component)
        if key not in self.approaches:
            self.approaches[key] = self.candidates(marker, box_offset, component)
        options = self.approaches[key]
        if not options:
            return None
        xy, yaw, distance = min(options, key=lambda item: np.linalg.norm(item[0] - start)
                                + 2 * abs(item[2] - 0.65))
        return dict(x=float(xy[0]), y=float(xy[1]), yaw=yaw)

    def candidates(self, marker, box_offset, component):
        target_field = np.asarray(self.markers[marker]) + box_offset
        target = self.xy(target_field)
        options = []
        for distance in (0.55, 0.65, 0.75):
            for angle in np.arange(16) * (2 * math.pi / 16):
                delta = distance * np.array([math.cos(angle), math.sin(angle)])
                xy = self.xy(target_field + delta)
                if self.cell(self.components, xy) != component:
                    continue
                ray = np.linspace(xy, target, 20, endpoint=False)
                if any(self.cell(self.clearance, p) <= 0 for p in ray):
                    continue
                yaw = math.atan2(target[1] - xy[1], target[0] - xy[0])
                options.append((xy, yaw, distance))
        return options

    def preview(self, path, start=None, goals=()):
        scale = 5
        preview = cv2.cvtColor(cv2.resize(self.image, None, fx=scale, fy=scale,
                                         interpolation=cv2.INTER_NEAREST), cv2.COLOR_GRAY2BGR)
        for marker, xy in self.markers.items():
            x, y = (p * scale + scale // 2 for p in self.pixel(self.xy(xy)))
            color = (180, 50, 0) if marker in ANCHORS else (70, 70, 70)
            cv2.circle(preview, (x, y), 3, color, -1)
            cv2.putText(preview, str(marker), (x + 4, y - 4), 0, 0.4, color, 1, cv2.LINE_AA)
        for pose in [*goals, *([start] if start else [])]:
            xy = np.array([pose["x"], pose["y"]])
            heading = np.array([math.cos(pose["yaw"]), math.sin(pose["yaw"])])
            a = tuple(p * scale + scale // 2 for p in self.pixel(xy))
            b = tuple(p * scale + scale // 2 for p in self.pixel(xy + heading * 0.35))
            cv2.arrowedLine(preview, a, b, (20, 20, 220) if pose is start else (0, 145, 0), 2)
        path.write_bytes(cv2.imencode(".png", preview)[1].tobytes())
