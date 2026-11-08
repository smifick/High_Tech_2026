import argparse
import os

from highteck.paths import SCENE_PATH

from highteck.simulation.drone_control import DroneFleet
from highteck.grid.mapping import get_base_xy

os.environ['SDL_VIDEO_WAYLAND_ALLOW_LIBDECOR'] = '0'
os.environ['SDL_HINT_VIDEO_WAYLAND_ALLOW_LIBDECOR'] = '0'

if 'GDK_BACKEND' in os.environ:
    del os.environ['GDK_BACKEND']

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", default=str(SCENE_PATH))
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    with DroneFleet(args.scene, realtime=not args.headless) as fleet:
        if not args.headless and fleet.viewer is None:
            fleet.launch_viewer()

        for i, drone in enumerate(fleet.drones):
            drone.control.navigate_wait(x=float(drone.position[0]), y=float(drone.position[1]), z=2.0, speed=2.0, frame_id="map", auto_arm=True)

            base_x, base_y = get_base_xy(drone, n=i + 1, h=3)
            print(f"Drone {i + 1}: {base_x}, {base_y}")

        if not args.headless and fleet.viewer is not None:
            print("Close the MuJoCo window to land all drones.")
            while fleet.viewer.is_running():
                fleet.step()

        for drone in fleet.drones:
            drone.control.land()
            drone.close()


if __name__ == "__main__":
    main()