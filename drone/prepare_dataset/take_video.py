from datetime import datetime
from pathlib import Path
from time import monotonic, sleep

import cv2
import sverk_interfaces

FPS = 15.0

folder = Path.home() / "videos"
folder.mkdir(parents=True, exist_ok=True)
filename = folder / f"video_{datetime.now():%Y%m%d_%H%M%S_%f}.mp4"

drone = sverk_interfaces.init(Nodename="video_recorder")
writer = None
frames = 0

try:
    while True:
        started = monotonic()
        image = drone.image.take_picture(timeout=1.0)

        if image is None:
            print("Кадр не получен", flush=True)
        else:
            if writer is None:
                height, width = image.shape[:2]
                writer = cv2.VideoWriter(
                    str(filename),
                    cv2.VideoWriter_fourcc(*"mp4v"),
                    FPS,
                    (width, height),
                )

                if not writer.isOpened():
                    raise RuntimeError(f"Не удалось открыть файл: {filename}")

                print(f"Запись: {filename} (Ctrl+C для остановки)", flush=True)

            writer.write(image)
            frames += 1

        sleep(max(0.0, 1.0 / FPS - (monotonic() - started)))

except KeyboardInterrupt:
    print("\nСъёмка остановлена")

finally:
    if writer is not None:
        writer.release()
    drone.close()

if frames:
    print(f"Сохранено: {filename} | кадров: {frames}")
else:
    print("Камера не прислала ни одного кадра")