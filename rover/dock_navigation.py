"""Lidar registration and route planning in the rover occupancy map."""

import heapq
import math

import cv2
import numpy as np


def pose(xy, yaw):
    return {"x": float(xy[0]), "y": float(xy[1]), "yaw": float(yaw)}


def xy(value):
    return np.array([value["x"], value["y"]])


def dock_localization(api, field, dock, radius, lidar_offset):
    """Match a stationary scan to mapped walls near the calibrated dock pose."""
    scan = api.get("/api/lidar/status", topic="/scan_filtered")
    if scan["frame_id"] != "lidar_link" or not scan["frame_ready"] or scan["age_sec"] > 0.5:
        raise RuntimeError("No fresh scan in lidar_link")
    points = -np.asarray(scan["points"], dtype=float) + lidar_offset
    points = points[np.isfinite(points).all(axis=1)]
    points = points[np.linalg.norm(points, axis=1) >= 0.22][::2]
    if len(points) < 65:
        raise RuntimeError("Too few lidar returns for dock localization")

    occupancy = 255 - field.image if field.info["negate"] else field.image
    walls = occupancy < 255 * (1 - field.info["occupied_thresh"])
    distances = cv2.distanceTransform((~walls).astype(np.uint8), cv2.DIST_L2, 5)
    distances = np.pad(distances * field.resolution, 1, constant_values=1.0)

    def search(center, offsets, yaw, angle_offsets):
        centers = np.array(np.meshgrid(offsets, offsets)).reshape(2, -1).T + center
        centers = np.array([p for p in centers if np.linalg.norm(p - xy(dock)) <= radius
                            and field.cell(field.clearance, p) > 0])
        if not len(centers):
            raise RuntimeError("Dock search area has no mapped free cells")
        candidates = []
        for angle in yaw + np.radians(angle_offsets):
            c, s = math.cos(angle), math.sin(angle)
            rotated = points @ np.array([[c, s], [-s, c]])
            grid = np.floor((centers[:, None, :] + rotated - field.origin) / field.resolution)
            grid = grid.astype(int)
            rows = np.clip(field.image.shape[0] - grid[:, :, 1], 0, distances.shape[0] - 1)
            cols = np.clip(grid[:, :, 0] + 1, 0, distances.shape[1] - 1)
            errors = distances[rows, cols]
            scores = np.minimum(errors, 0.30).mean(axis=1)
            for i in np.argsort(scores)[:6]:
                candidates.append((float(scores[i]), pose(centers[i], angle),
                                   float((errors[i] < 0.14).mean())))
        return sorted(candidates, key=lambda candidate: candidate[0])

    coarse = search(xy(dock), np.arange(-radius, radius + .04, .07),
                    dock["yaw"], np.arange(-25, 26, 5))
    score, result, fraction = search(xy(coarse[0][1]), np.arange(-.075, .08, .025),
                                     coarse[0][1]["yaw"], np.arange(-5, 6))[0]
    ambiguous = any(candidate_score < score + 0.012
                    and (np.linalg.norm(xy(candidate) - xy(result)) > .20
                         or abs(candidate["yaw"] - result["yaw"]) > math.radians(12))
                    for candidate_score, candidate, _ in coarse)
    if fraction < .68 or score > .115 or ambiguous:
        raise RuntimeError(f"Dock localization not confirmed (wall matches {fraction:.0%}, "
                           f"error {score:.2f} m, ambiguous={ambiguous})")
    return result, {"wall_match_fraction": fraction, "mean_clipped_error_m": score}


class Routes:
    def __init__(self, field, boxes, box_radius, clearance):
        self.field = field
        self.free = field.clearance >= clearance
        self.boxes = boxes
        self.keepout = box_radius + clearance
        rows, cols = np.indices(self.free.shape)
        x = field.origin[0] + (cols + .5) * field.resolution
        y = field.origin[1] + (self.free.shape[0] - rows - .5) * field.resolution
        for bx, by in boxes.values():
            self.free &= (x - bx)**2 + (y - by)**2 >= self.keepout**2

    def allows(self, a, b):
        distance = float(np.linalg.norm(b - a))
        # Also check the two cells flanking diagonal transitions.
        pixels = [self.field.pixel(point) for point in
                  np.linspace(a, b, max(2, math.ceil(distance / (.35 * self.field.resolution)) + 1))]
        prev = pixels[0]
        for x, y in pixels:
            if not self.cell(x, y):
                return False
            if x != prev[0] and y != prev[1]:
                if not self.cell(x, prev[1]) or not self.cell(prev[0], y):
                    return False
            prev = (x, y)
        return True

    def cell(self, x, y):
        return 0 <= y < self.free.shape[0] and 0 <= x < self.free.shape[1] and self.free[y, x]

    def path(self, a, b):
        start, end = self.field.pixel(a), self.field.pixel(b)
        if not self.cell(*start) or not self.cell(*end):
            return None
        if self.allows(a, b):
            return [b]
        queue = [(0., 0., start)]
        costs = {start: 0.}
        parent = {}
        while queue:
            _, cost, cell = heapq.heappop(queue)
            if cell == end:
                break
            if cost > costs[cell]:
                continue
            x, y = cell
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
                next_cell = (x+dx,y+dy)
                if not self.cell(*next_cell) or cost + 1 >= costs.get(next_cell, math.inf):
                    continue
                parent[next_cell] = cell
                costs[next_cell] = cost + 1
                priority = cost + 1 + abs(end[0]-next_cell[0]) + abs(end[1]-next_cell[1])
                heapq.heappush(queue, (priority, cost+1, next_cell))
        if end not in parent:
            return None
        chain = [b]
        cell = end
        while cell != start:
            cell = parent[cell]
            if cell != start:
                chain.append(self.field.world(cell))
        chain.reverse()
        simplified = []
        position = a
        while chain:
            # Prefer a long straight segment when the whole segment has clearance.
            for i in range(len(chain)-1, -1, -1):
                if self.allows(position, chain[i]):
                    position = chain[i]
                    simplified.append(position)
                    del chain[:i+1]
                    break
            else:
                return None
        return simplified

    def waypoints(self, start, goal):
        path = self.path(xy(start), xy(goal))
        if path is None:
            return None
        return [pose(a, math.atan2(*(b-a)[::-1])) for a,b in zip(path, path[1:])] + [goal]

    def view(self, marker, axis, origin, distance):
        center = self.boxes[marker]
        point = center + distance * axis
        delta = center - point
        goal = pose(point, math.atan2(delta[1], delta[0]))
        # The camera ray may enter the target's keep-out disc, but must clear other boxes.
        for p in np.linspace(point, center, 24):
            if self.field.cell(self.field.clearance, p) <= 0:
                return None
            if any(np.linalg.norm(p-other) < self.keepout / 2
                   for i, other in self.boxes.items() if i != marker):
                return None
        return goal if self.path(xy(origin), point) is not None else None
