import numpy as np


def match_box_to_marker(box_points, markers, angle_deg=0):
    box = np.asarray(box_points, dtype=float)

    if box.shape != (4, 2):
        raise ValueError("Нужны четыре точки коробки: shape (4, 2)")

    if not np.isfinite(box).all() or not np.isfinite(angle_deg):
        raise ValueError("Координаты и угол должны быть конечными")

    center = box.mean(axis=0)
    angle = np.deg2rad(angle_deg)

    rotation = np.array([
        [np.cos(angle), -np.sin(angle)],
        [np.sin(angle),  np.cos(angle)],
    ])

    local_box = (box - center) @ rotation

    x_left, y_top = local_box.min(axis=0)
    x_right, y_bottom = local_box.max(axis=0)

    width = x_right - x_left

    if width <= 0 or y_bottom <= y_top:
        raise ValueError("Коробка должна иметь ненулевые ширину и высоту")

    best_id = None
    best_distance = float("inf")

    for marker_id, marker_center in markers.items():
        point = np.asarray(marker_center, dtype=float)

        if point.shape != (2,) or not np.isfinite(point).all():
            raise ValueError(f"Некорректный центр метки {marker_id}")

        x, y = (point - center) @ rotation
        gap = x_left - x

        if not (0 <= gap <= 0.6 * width):
            continue

        if not (y_top <= y <= y_bottom):
            continue

        distance = np.hypot(gap, y - (y_top + y_bottom) / 2)

        if distance < best_distance:
            best_distance = distance
            best_id = marker_id

    return best_id