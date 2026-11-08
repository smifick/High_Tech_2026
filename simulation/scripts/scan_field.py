import argparse
import os
from time import sleep

import cv2

from grid.mapping import GRID_STEP
from simulation.logic.paths import SCENE_PATH
from simulation.logic.simulation.drone_control import DroneFleet
from simulation.logic.grid.mapping import get_base_xy

os.environ['SDL_VIDEO_WAYLAND_ALLOW_LIBDECOR'] = '0'
os.environ['SDL_HINT_VIDEO_WAYLAND_ALLOW_LIBDECOR'] = '0'

if 'GDK_BACKEND' in os.environ:
    del os.environ['GDK_BACKEND']

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", default=str(SCENE_PATH))
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    base = []

    with DroneFleet(args.scene, realtime=not args.headless) as fleet:
        if not args.headless and fleet.viewer is None:
            fleet.launch_viewer()

        for i, drone in enumerate(fleet.drones):
            drone.control.navigate_wait(x=float(drone.position[0]), y=float(drone.position[1]), z=2.0, speed=2.0, frame_id="map", auto_arm=True)

            base_x, base_y = get_base_xy(drone, n=i + 1, h=4)
            print(f"Drone {i + 1}: {base_x}, {base_y}")
            base.append((base_x, base_y))

        sleep(2)
        print('Старт обхода')
        points = [(2, 2), (-2, 2), (2, -2), (-2, -2)]

        if len(fleet.drones) != 4:
            raise ValueError("Этот обход рассчитан на четыре дрона")

        # Общие вершины квадрата в координатах map.
        vertices = [
            (
                base[i][0] + dx * GRID_STEP,
                base[i][1] + dy * GRID_STEP,
            )
            for i, (dx, dy) in enumerate(points)
        ]

        order = [0, 1, 3, 2]

        for j in range(5):
            for i, drone in enumerate(fleet.drones):
                start_index = order.index(i)
                vertex_index = order[(start_index + j) % 4]

                target_x, target_y = vertices[vertex_index]

                drone.control.navigate(
                    x=target_x,
                    y=target_y,
                    z=2.0,
                    speed=0.6,
                    frame_id="map",
                )
            for i, drone in enumerate(fleet.drones):
                image = drone.get_image(width=640, height=480)

                cv2.imshow("Drone Camera", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
                cv2.waitKey(0)
                cv2.destroyAllWindows()

            while any(drone.control.moving for drone in fleet.drones):
                fleet.step()

        if not args.headless and fleet.viewer is not None:
            print("Close the MuJoCo window to land all drones.")
            while fleet.viewer.is_running():
                fleet.step()

        for drone in fleet.drones:
            drone.control.land()
            drone.close()


if __name__ == "__main__":
    main()