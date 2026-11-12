import argparse
from pathlib import Path

from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("video", type=Path, help="Путь к исходному видео")
    parser.add_argument("--weights", type=Path, default=Path("best.pt"))
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--device", default=None, help="Например, 0 или cpu")
    args = parser.parse_args()

    for path in (args.video, args.weights):
        if not path.is_file():
            parser.error(f"Файл не найден: {path}")

    model = YOLO(str(args.weights))
    results = model.predict(
        source=str(args.video),
        conf=args.conf,
        device=args.device,
        stream=True,
        save=True,
        project="runs",
        name="boxes",
        verbose=False,
        show=True,
    )

    frames = 0
    for result in results:
        frames += 1

    if frames == 0:
        raise RuntimeError("Не удалось прочитать ни одного кадра видео")

    print(f"Обработано кадров: {frames}")
    print(f"Результат: {result.save_dir}")


if __name__ == "__main__":
    main()