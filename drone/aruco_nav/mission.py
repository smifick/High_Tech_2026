"""Validation shared by the station and onboard client. Coordinates are metres."""
import json
import math
from pathlib import Path

MIN_ROUTE_GAP_M = 0.75


def number(value, name, low=None, high=None):
    if isinstance(value, bool) or not isinstance(value, (float, int)):
        raise ValueError(f'{name}: требуется число')
    value = float(value)
    if not math.isfinite(value) or (low is not None and value < low) or (high is not None and value > high):
        raise ValueError(f'{name}: недопустимое значение')
    return value


def route(value):
    if not isinstance(value, dict) or set(value) - {'height', 'speed', 'points'}:
        raise ValueError('Маршрут: допустимы height, speed, points')
    height = number(value.get('height', 1.7), 'height', 0.2, 5)
    speed = number(value.get('speed', 0.3), 'speed', 0.05, 1)
    if speed != 0.3:
        raise ValueError('Исходный клиент поддерживает только speed=0.3')
    points = value.get('points')
    if not isinstance(points, list) or not 1 <= len(points) <= 1000:
        raise ValueError('points должен содержать 1..1000 точек [x, y]')
    checked = []
    for p in points:
        if not isinstance(p, list) or len(p) != 2:
            raise ValueError('Точка должна иметь формат [x, y]')
        checked.append([number(p[0], 'x'), number(p[1], 'y')])
    return {'height': height, 'speed': speed, 'points': checked}


def point_distance(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(dx, dy)
    if not length:
        return math.dist(p, a)
    along = (p[0] - a[0]) * (dx / length) + (p[1] - a[1]) * (dy / length)
    t = min(1.0, max(0.0, along / length))
    return math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy))


def orientation(a, b, p):
    return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])


def segment_distance(a, b, c, d):
    if (orientation(a, b, c) * orientation(a, b, d) < 0
            and orientation(c, d, a) * orientation(c, d, b) < 0):
        return 0.0
    return min(point_distance(a, c, d), point_distance(b, c, d),
               point_distance(c, a, b), point_distance(d, a, b))


def validate_routes(routes):
    if not isinstance(routes, dict) or len(routes) != 2:
        raise ValueError('Нужны маршруты ровно двух дронов')
    if any(not isinstance(key, str) or not key.strip() or len(key) > 100 for key in routes):
        raise ValueError('Неверный ID дрона')
    checked = {key: route(value) for key, value in routes.items()}
    first, second = (spec['points'] for spec in checked.values())
    left = list(zip(first, first[1:])) or [(first[0], first[0])]
    right = list(zip(second, second[1:])) or [(second[0], second[0])]
    for i, (a, b) in enumerate(left, 1):
        for j, (c, d) in enumerate(right, 1):
            if segment_distance(a, b, c, d) < MIN_ROUTE_GAP_M:
                names = tuple(checked)
                raise ValueError(f'{names[0]} участок {i} и {names[1]} участок {j}: '
                                 f'горизонтальный зазор меньше {MIN_ROUTE_GAP_M:.2f} м')
    return checked


def load_mission(path):
    data = json.loads(Path(path).read_text(encoding='utf-8-sig'))
    if not isinstance(data, dict) or set(data) != {'routes'}:
        raise ValueError('Ожидается объект routes')
    return validate_routes(data['routes'])
