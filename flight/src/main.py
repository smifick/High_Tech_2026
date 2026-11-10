from time import sleep

import sverk_interfaces

drone = sverk_interfaces.init(Nodename="fly_scan")

try:
    drone.control.navigate_wait(
        x=0.0,
        y=0.0,
        z=1.0,
        frame_id="body",
        auto_arm=True,
        speed=0.3,
        tolerance=0.15,
        timeout=30.0,
    )

    telemetry = drone.control.get_telemetry(frame_id="map")

    if not telemetry.connected or not telemetry.armed:
        raise RuntimeError("Нет связи с контроллером или моторы не включены")

    drone.control.navigate_wait(
        x=3.70,
        y=0.0,
        z=0.0,
        yaw=0.0,
        frame_id="body",
        speed=0.3,
        tolerance=0.15,
        timeout=60.0,
    )

    sleep(2)

    drone.control.land()

finally:
    drone.control.land()
    drone.close()
