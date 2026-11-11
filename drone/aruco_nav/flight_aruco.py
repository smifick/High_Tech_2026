import sverk_interfaces

drone = sverk_interfaces.init(Nodename="fly_scan")

try:
    drone.control.navigate_wait(
        x=0.0,
        y=0.0,
        z=1.7,
        frame_id="map",
        auto_arm=True,
        speed=0.3,
        tolerance=0.15,
        timeout=30.0,
    )

    telemetry = drone.control.get_telemetry(frame_id="map")

    if not telemetry.connected or not telemetry.armed:
        raise RuntimeError("Нет связи с контроллером или моторы не включены")

    drone.control.navigate_wait(
        x=2.0,
        y=2.0,
        z=1.7,
        yaw=0.0,
        frame_id="aruco_map",
        speed=0.3,
        tolerance=0.15,
        timeout=60.0,
    )

    drone.control.land()

finally:
    drone.control.land()
    drone.close()
