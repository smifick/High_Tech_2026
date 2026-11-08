#!/usr/bin/env python3
"""Generate a MuJoCo field for the HiTech 2026 inventory challenge (case 1)."""
import argparse

import json
import random
from pathlib import Path
from xml.etree import ElementTree as ET

import cv2

from highteck.paths import SCENE_PATH

N = 10
STEP = 0.85  # The specification permits 0.70–1.00 m.
MARKER_SIDE = 0.18
BOX_SIDE = 0.40  # >= 0.35 m
BOX_HEIGHT = 0.35
BOX_OFFSET_X = 0.31  # Marker on the left, cargo on the right.


def generate(output: Path, seed: int, occupied_count: int, drones: int = 4):
    if not 1 <= drones <= 4:
        raise ValueError("drones must be between 1 and 4")
    if not 5 <= occupied_count <= 15:
        raise ValueError("occupied_count must be between 5 and 15")
    output.mkdir(parents=True, exist_ok=True)
    tex_dir = output / "textures"
    tex_dir.mkdir(exist_ok=True)

    rng = random.Random(seed)
    occupied = set(rng.sample(range(100), occupied_count))
    damaged = set(rng.sample(sorted(occupied), 5))

    mujoco = ET.Element("mujoco", model="hitech_aruco_inventory")
    ET.SubElement(mujoco, "compiler", angle="radian")
    ET.SubElement(mujoco, "option", timestep="0.01", gravity="0 0 -9.81")
    ET.SubElement(mujoco, "visual")
    asset = ET.SubElement(mujoco, "asset")
    ET.SubElement(asset, "material", name="floor_mat", rgba="0.64 0.67 0.69 1")
    ET.SubElement(asset, "material", name="cargo_mat", rgba="0.70 0.52 0.32 1")
    ET.SubElement(asset, "material", name="damage_mat", rgba="0.35 0.14 0.12 1")
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    for marker_id in range(100):
        # Explicit per-marker texture means the camera sees a genuine ArUco bit pattern.
        pixels = cv2.aruco.generateImageMarker(dictionary, marker_id, 256)
        cv2.imwrite(str(tex_dir / f"aruco_{marker_id:02d}.png"), pixels)
        ET.SubElement(asset, "texture", name=f"aruco_tex_{marker_id:02d}",
                      type="2d", file=f"textures/aruco_{marker_id:02d}.png")
        ET.SubElement(asset, "material", name=f"aruco_mat_{marker_id:02d}",
                      texture=f"aruco_tex_{marker_id:02d}", texuniform="false",
                      rgba="1 1 1 1", specular="0", shininess="0")

    world = ET.SubElement(mujoco, "worldbody")
    ET.SubElement(world, "light", name="sun", pos="0 0 12", dir="0 0 -1",
                  directional="true", diffuse="0.9 0.9 0.9")
    ET.SubElement(world, "geom", name="ground", type="plane", size="6 6 0.1",
                  material="floor_mat")
    ET.SubElement(world, "camera", name="top_down", pos="0 0 13",
                  xyaxes="1 0 0 0 1 0", fovy="43")
    ET.SubElement(world, "camera", name="rover_view", pos="0 -5 0.7",
                  xyaxes="1 0 0 0 0.14 0.99", fovy="75")
    # Kinematic (mocap) drone prototypes: independently positionable, with nadir cameras.
    # Cameras are children of the drone body and move with it.
    for drone_id in range(drones):
        start_x = (drone_id - (drones - 1) / 2) * 0.9
        drone = ET.SubElement(world, "body", name=f"drone_{drone_id}",
                              mocap="true", pos=f"{start_x:.3f} -4.5 0.25")
        ET.SubElement(drone, "geom", name=f"drone_{drone_id}_body", type="box",
                      size="0.12 0.12 0.045", rgba="0.15 0.30 0.77 1",
                      contype="0", conaffinity="0")
        for arm_x, arm_y in [(0.17, 0.17), (0.17, -0.17), (-0.17, 0.17), (-0.17, -0.17)]:
            ET.SubElement(drone, "geom", name=f"drone_{drone_id}_motor_{'p' if arm_x>0 else 'm'}{'p' if arm_y>0 else 'm'}",
                          type="sphere", size="0.045", pos=f"{arm_x} {arm_y} 0",
                          rgba="0.1 0.1 0.14 1", contype="0", conaffinity="0")
        ET.SubElement(drone, "camera", name=f"drone_{drone_id}_down",
                      pos="0 0 -0.06", xyaxes="1 0 0 0 1 0", fovy="70")
    inventory = []

    for marker_id in range(100):
        row, col = divmod(marker_id, N)
        x = (col - (N - 1) / 2) * STEP
        y = ((N - 1) / 2 - row) * STEP
        marker = ET.SubElement(world, "body", name=f"slot_{marker_id:02d}",
                               pos=f"{x:.4f} {y:.4f} 0")
        # Ground is at z=0. Keep textured marker slightly above it to avoid z-fighting.
        ET.SubElement(marker, "geom", name=f"marker_{marker_id:02d}",
                      type="box", size=f"{MARKER_SIDE/2} {MARKER_SIDE/2} 0.001",
                      pos="0 0 0.003", material=f"aruco_mat_{marker_id:02d}",
                      contype="0", conaffinity="0")
        state = "empty"
        if marker_id in occupied:
            state = "damaged" if marker_id in damaged else "occupied"
            cargo = ET.SubElement(world, "body", name=f"cargo_{marker_id:02d}",
                                  pos=f"{x + BOX_OFFSET_X:.4f} {y:.4f} {BOX_HEIGHT/2:.4f}")
            ET.SubElement(cargo, "geom", name=f"box_{marker_id:02d}",
                          type="box", size=f"{BOX_SIDE/2} {BOX_SIDE/2} {BOX_HEIGHT/2}",
                          material="cargo_mat")
            if marker_id in damaged:
                # Side-only damage: visible from a lateral camera, not from above.
                ET.SubElement(cargo, "geom", name=f"damage_{marker_id:02d}",
                              type="box", pos=f"0 {-BOX_SIDE/2 - 0.003:.4f} 0",
                              size="0.105 0.004 0.075", material="damage_mat",
                              contype="0", conaffinity="0")
        inventory.append({"marker_id": marker_id, "x": round(x, 4),
                          "y": round(y, 4), "state": state})

    # Use minimal indentation to keep file browsable.
    ET.indent(mujoco, space="  ")
    (output / "field.xml").write_text(ET.tostring(mujoco, encoding="unicode"), encoding="utf-8")
    (output / "ground_truth.json").write_text(
        json.dumps({"seed": seed, "dictionary": "DICT_4X4_100", "step_m": STEP, "drone_count": drones,
                    "slots": inventory}, ensure_ascii=False, indent=2),
        encoding="utf-8")
    print(f"Generated: {output / 'field.xml'} | 100 markers, {occupied_count} boxes, {drones} drones, 5 side anomalies")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=SCENE_PATH.parent)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--boxes", type=int, default=15)
    parser.add_argument("--drones", type=int, default=2)
    args = parser.parse_args()
    generate(args.output, args.seed, args.boxes, args.drones)
