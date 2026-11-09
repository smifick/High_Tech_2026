import argparse
from pathlib import Path

from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("image", help="Путь к картинке")
    parser.add_argument("--conf", type=float, default=0.25)
    args = parser.parse_args()

    image = Path(args.image)
    if not image.is_file():
        parser.error(f"Не найдена картинка: {image}")

    # Загружаем именно ваши обученные веса
    folder = Path(__file__).resolve().parent
    model = YOLO(str(folder / "best.pt"))

    result = model.predict(
        source=str(image),
        conf=args.conf,
        device="cpu",
        verbose=False,
    )[0]

    count = len(result.boxes)
    print(f"Найдено коробок: {count}" if count else "Коробка не обнаружена")

    for box in result.boxes:
        confidence = float(box.conf.item())
        coordinates = box.xyxy[0].tolist()
        print(f"Уверенность: {confidence:.3f}, рамка: {coordinates}")

    output = folder / "results"
    output.mkdir(exist_ok=True)
    saved_image = output / f"{image.stem}_detected.jpg"
    result.save(filename=str(saved_image))
    print(f"Картинка с рамками сохранена: {saved_image}")


if __name__ == "__main__":
    main()
