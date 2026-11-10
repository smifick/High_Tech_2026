from datetime import datetime
from pathlib import Path

import cv2
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="save_photo")

try:
    folder = Path("photos")
    folder.mkdir(parents=True, exist_ok=True)

    image = drone.image.take_picture(timeout=5.0)

    if image is None:
        print("Не удалось получить кадр с камеры")
    else:
        filename = folder / f"photo_{datetime.now():%Y%m%d_%H%M%S_%f}.jpg"

        if cv2.imwrite(str(filename), image):
            print(f"Фото сохранено: {filename.resolve()}")
        else:
            print("Ошибка сохранения фотографии")
except:
    drone.control.land()
    drone.close()
finally:
    drone.control.land()
    drone.close()
