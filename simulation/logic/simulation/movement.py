"""Синхронное перемещение дронов в общей симуляции."""
import math
import numpy as np

def navigate_together(drones, targets, speed=1.5):
    """Двигать mocap-дроны одновременно, вызывая один шаг общей симуляции."""
    if not drones or len(drones) != len(targets):
        raise ValueError("Provide one target per drone")
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError("speed must be finite and positive")
    fleet = drones[0].fleet
    if any(drone.fleet is not fleet for drone in drones):
        raise ValueError("Drones must belong to the same fleet")
    targets = np.asarray(targets, dtype=float)
    if targets.shape != (len(drones), 3) or not np.isfinite(targets).all() or (targets[:, 2] < 0.15).any():
        raise ValueError("Targets must be finite xyz positions at altitude >= 0.15")
    starts = np.array([drone.position for drone in drones])
    deltas = targets - starts
    distances = np.linalg.norm(deltas, axis=1)
    steps = max(1, math.ceil(float(distances.max()) / (speed * fleet.dt)))
    for drone in drones:
        drone.armed = True
    for step in range(1, steps + 1):
        if fleet.viewer is not None and not fleet.viewer.is_running():
            raise InterruptedError("Viewer closed")
        fractions = np.minimum(1, step * fleet.dt * speed / np.maximum(distances, 1e-12))
        for drone, position in zip(drones, starts + fractions[:, None] * deltas):
            drone.position = position
        fleet.step()
