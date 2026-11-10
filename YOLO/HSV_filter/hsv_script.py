from pathlib import Path

import cv2
import numpy as np


# Папка с оригинальными фотографиями рядом со скриптом.
IMAGES_DIR = Path(__file__).resolve().parent / "assets"

LOWER = np.array([90, 100, 40], dtype=np.uint8)
UPPER = np.array([110, 255, 255], dtype=np.uint8)


def main():
    if not IMAGES_DIR.is_dir():
        raise FileNotFoundError(f"Папка не найдена: {IMAGES_DIR}")

    # Фиксируем список до создания результатов.
    # Готовые маски и результаты повторно не обрабатываем.
    paths = sorted(
        path for path in IMAGES_DIR.iterdir()
        if path.is_file()
        and not path.stem.lower().endswith(
            ("__hsv", "__mask", "_mask", "_result", "_filtered")
        )
    )

    processed = 0

    for path in paths:
        try:
            content = np.frombuffer(
                path.read_bytes(),
                dtype=np.uint8,
            )

            if content.size == 0:
                continue

            # Пробуем прочитать изображение независимо от расширения.
            image = cv2.imdecode(content, cv2.IMREAD_COLOR)

        except (OSError, cv2.error) as error:
            print(f"Пропущен {path.name}: {error}")
            continue

        if image is None:
            print(f"Пропущен {path.name}: не удалось прочитать изображение")
            continue

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, LOWER, UPPER)

        kernel = np.ones((3, 3), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Сохраняем рядом с оригиналом.
        # Исходное расширение включено, чтобы a.jpg и a.png
        # не получили одинаковые имена результатов.
        hsv_path = path.with_name(f"{path.name}__hsv.png")
        mask_path = path.with_name(f"{path.name}__mask.png")

        if not cv2.imwrite(str(hsv_path), hsv):
            raise RuntimeError(f"Не удалось сохранить {hsv_path}")

        if not cv2.imwrite(str(mask_path), mask):
            raise RuntimeError(f"Не удалось сохранить {mask_path}")

        processed += 1
        print(f"Готово: {path.name}")

    print(f"Обработано изображений: {processed}")


if __name__ == "__main__":
    main()

