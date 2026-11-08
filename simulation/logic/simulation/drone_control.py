from dataclasses import dataclass
from pathlib import Path
import math
import time
import mujoco
import numpy as np

from highteck.paths import SCENE_PATH

import os
os.environ["XDG_SESSION_TYPE"] = "x11"



@dataclass(frozen=True)
class Telemetry:
    x: float
    y: float
    z: float
    armed: bool
    flying: bool


class DroneControl:
    def __init__(self, drone: "Drone"):
        self.drone = drone

    def get_telemetry(self) -> Telemetry:
        x, y, z = self.drone.position
        return Telemetry(float(x), float(y), float(z), self.drone.armed, self.drone.armed and z > 0.35)

    def navigate_wait(self, x=0.0, y=0.0, z=0.0, speed=0.6,
                      frame_id="map", auto_arm=False):
        if speed <= 0:
            raise ValueError("speed must be positive")
        if frame_id not in ("map", "body"):
            raise ValueError("frame_id must be 'map' or 'body'")
        if auto_arm:
            self.drone.armed = True
        if not self.drone.armed:
            raise RuntimeError("Drone is not armed: use auto_arm=True on first command")
        start = self.drone.position.copy()
        target = np.array([x, y, z], dtype=float)
        if frame_id == "body":
            target = start + target  # Orientation is fixed in this simple simulator.
        if target[2] < 0.15:
            raise ValueError("Target altitude must be >= 0.15 m; use land() to descend")
        delta = target - start
        distance = float(np.linalg.norm(delta))
        if distance == 0:
            return self.get_telemetry()
        duration = distance / speed
        dt = self.drone.fleet.dt
        steps = max(1, math.ceil(duration / dt))
        for step in range(1, steps + 1):
            alpha = step / steps
            self.drone.position = start + alpha * delta
            self.drone.fleet.step()
        return self.get_telemetry()

    def land(self, speed=0.4):
        if speed <= 0:
            raise ValueError("speed must be positive")
        start = self.drone.position.copy()
        goal_z = 0.15
        steps = max(1, math.ceil(abs(start[2] - goal_z) / (speed * self.drone.fleet.dt)))
        for step in range(1, steps + 1):
            pos = start.copy()
            pos[2] = start[2] + (goal_z - start[2]) * step / steps
            self.drone.position = pos
            self.drone.fleet.step()
        self.drone.armed = False
        return self.get_telemetry()


class Drone:
    def __init__(self, fleet: "DroneFleet", drone_id: int):
        self.fleet = fleet
        self.id = drone_id
        name = f"drone_{drone_id}"
        body_id = mujoco.mj_name2id(fleet.model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id < 0:
            raise ValueError(f"Unknown drone {drone_id}; regenerate field with --drones {drone_id+1}")
        self.mocap_id = int(fleet.model.body_mocapid[body_id])
        self.camera_name = f"{name}_down"
        self.armed = False
        self.control = DroneControl(self)

    @property
    def position(self) -> np.ndarray:
        return self.fleet.data.mocap_pos[self.mocap_id].copy()

    @position.setter
    def position(self, value):
        self.fleet.data.mocap_pos[self.mocap_id] = np.asarray(value, dtype=float)

    def get_image(self, width=640, height=480) -> np.ndarray:
        return self.fleet.get_image(self.id, width, height)

    def close(self):
        pass


class DroneFleet:
    def __init__(self, scene_path=SCENE_PATH, realtime=False):
        self.model = mujoco.MjModel.from_xml_path(str(Path(scene_path).resolve()))
        self.data = mujoco.MjData(self.model)
        self.dt = float(self.model.opt.timestep)
        self.realtime = realtime
        self.viewer = None
        self.drones = []
        drone_id = 0
        while mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, f"drone_{drone_id}") >= 0:
            self.drones.append(Drone(self, drone_id))
            drone_id += 1
        mujoco.mj_forward(self.model, self.data)

    def __len__(self):
        return len(self.drones)

    def __getitem__(self, index):
        return self.drones[index]

    def step(self):
        mujoco.mj_step(self.model, self.data)
        if self.viewer is not None and self.viewer.is_running():
            self.viewer.sync()
        if self.realtime:
            time.sleep(self.dt)

    def get_image(self, drone_id, width=640, height=480):
        if width <= 0 or height <= 0:
            raise ValueError("Image dimensions must be positive")
        mujoco.mj_forward(self.model, self.data)
        # One temporary renderer per capture; use a persistent renderer for high frame rates.
        with mujoco.Renderer(self.model, height=height, width=width) as renderer:
            renderer.update_scene(self.data, camera=self[drone_id].camera_name)
            return renderer.render().copy()

    def launch_viewer(self):
        import mujoco.viewer
        if self.viewer is not None:
            raise RuntimeError("Viewer already open")
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        return self.viewer

    def close(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
