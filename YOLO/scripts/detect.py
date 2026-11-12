import argparse
from pathlib import Path

from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("image", help="Путь к картинке")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument('--weights', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'YOLO11n_models/YOLO11n_baseline/weights/best_drone_photos_yolo.pt')
    args = parser.parse_args()
    if not args.weights.is_file():
        parser.error(f'Веса не найдены: {args.weights}')

    image = Path(args.image)
    if not image.is_file():
        parser.error(f"Не найдена картинка: {image}")

    folder = Path(__file__).resolve().parent
    model = YOLO(str(args.weights))

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
