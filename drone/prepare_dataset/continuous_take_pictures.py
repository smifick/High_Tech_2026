from datetime import datetime
from pathlib import Path
from time import monotonic, sleep

import cv2
import sverk_interfaces

folder = Path.home() / "photos"
folder.mkdir(parents=True, exist_ok=True)

drone = sverk_interfaces.init(Nodename="photo_timer")

try:
    next_photo = monotonic()

    while True:
        sleep(max(0, next_photo - monotonic()))

        image = drone.image.take_picture(timeout=5.0)

        if image is None:
            print("Кадр не получен", flush=True)
        else:
            filename = folder / f"photo_{datetime.now():%Y%m%d_%H%M%S_%f}.jpg"

            if cv2.imwrite(str(filename), image):
                print(f"Сохранено: {filename}", flush=True)
            else:
                print(f"Ошибка сохранения: {filename}", flush=True)

        next_photo += 15.0

        while next_photo < monotonic():
            next_photo += 15.0

except KeyboardInterrupt:
    print("\nСъёмка остановлена")
finally:
    drone.close()
