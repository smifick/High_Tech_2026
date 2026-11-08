from simulation.logic.grid.storage import save_grid
from simulation.logic.vision.markers import WIDTH, HEIGHT, get_markers, nearest_marker


GRID_STEP = 0.85
TOLERANCE = 10

def get_base_xy(drone, n=1, h=2):
    """n = {1 RL, 2 RR, 3 FL, 4 FR}"""
    step = 0.05
    x = drone.position[0]
    y = drone.position[1]
    for _ in range(400):
        centers, ids, rejected = get_markers(drone)

        if not centers:
            raise RuntimeError("No markers visible while locating grid origin")

        cond1, cond2, direction = get_angel_n(centers, n)

        if cond1:
            y+=step
        elif cond2:
            x+=step * direction
        else:
            return x,y
        drone.control.navigate_wait(x=x, y=y, z=h, speed=15.0, frame_id="map", auto_arm=True)
    raise RuntimeError("Could not locate grid origin in 400 steps")

def get_angel_n(centers, n=1) -> tuple:
    """n = {1 RL, 2 RR, 3 FL, 4 FR}"""
    H = (HEIGHT / 2) + 10
    W = (WIDTH / 2) + 10

    try:
        match(n):
            case 1:
                direction = -1
                return (
                    max(centers, key=lambda x: x[1])[1] < H,
                    min(centers, key=lambda x: x[0])[0] < W,
                    direction
                )
            case 2:
                direction = 1
                return (
                    max(centers, key=lambda x: x[1])[1] < H,
                    max(centers, key=lambda x: x[0])[0] > W,
                    direction
                )
            case 3:
                direction = -1
                return (
                    min(centers, key=lambda x: x[1])[1] < H,
                    min(centers, key=lambda x: x[0])[0] < W,
                    direction
                )
            case 4:
                direction = 1
                return (
                    min(centers, key=lambda x: x[1])[1] < H,
                    max(centers, key=lambda x: x[0])[0] > W,
                    direction
                )

    except RuntimeError:
        return True, True, direction

def get_grid(drone, rows, cols, output_file="grid.json", n=1):
    """n = {1 RL, 2 RR, 3 FL, 4 FR}"""
    base_x, base_y = get_base_xy(drone, n)
    result = []

    for row in range(rows):
        for col in range(cols):
            grid_col = col if row % 2 == 0 else cols - col - 1

            x = grid_col * GRID_STEP
            y = row * GRID_STEP

            drone.control.navigate_wait(x=x + base_x, y=y + base_y, z=2.0, speed=1.5, frame_id="map", auto_arm=True)

            marker_id = nearest_marker(drone)

            result.append({
                "row": row,
                "col": grid_col,
                "x": x,
                "y": y,
                "marker_id": marker_id
            })

            print(f"Cell [{row}, {grid_col}]: marker={marker_id}")

            save_grid(result, output_file)

    return result
