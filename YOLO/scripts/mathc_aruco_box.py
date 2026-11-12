"""Reusable YOLO + ArUco matcher for images and live video."""
import argparse
from pathlib import Path
import sys

import cv2
import numpy as np
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from shared.match_box import match_box_to_marker

DEFAULT_WEIGHTS = ROOT / 'YOLO/YOLO11n_models/YOLO11n_finetune/best_for_finetune.pt'


class FrameProcessor:
    def __init__(self, weights=DEFAULT_WEIGHTS, device='cpu', dictionary='DICT_4X4_250', conf=0.25):
        if not Path(weights).is_file():
            raise FileNotFoundError(f'Веса не найдены: {weights}')
        if not dictionary.startswith('DICT_') or not hasattr(cv2.aruco, dictionary):
            raise ValueError(f'Неизвестный словарь ArUco: {dictionary}')
        self.model = YOLO(str(weights))
        self.device = device
        self.conf = conf
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary)),
            cv2.aruco.DetectorParameters(),
        )

    def process(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        markers = {} if ids is None else {
            int(marker_id): corner.reshape(4, 2).mean(axis=0).tolist()
            for marker_id, corner in zip(ids.flatten(), corners)
        }
        result = self.model.predict(source=img, device=self.device, conf=self.conf, verbose=False)[0]
        matches = []
        output = img.copy()

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(output, corners, ids)

            for marker_id, marker_center in markers.items():
                center = tuple(np.rint(marker_center).astype(int))
                cv2.circle(output, center, 4, (0, 255, 0), -1)

        for index, (x1, y1, x2, y2) in enumerate(
            result.boxes.xyxy.cpu().numpy(),
            start=1,
        ):
            box_points = [
                [float(x1), float(y1)],
                [float(x2), float(y1)],
                [float(x2), float(y2)],
                [float(x1), float(y2)],
            ]

            marker_id = match_box_to_marker(
                box_points,
                markers,
                angle_deg=0,
            )

            box_center = (
                int(round((x1 + x2) / 2)),
                int(round((y1 + y2) / 2)),
            )

            if marker_id is not None:
                color = (255, 0, 0)
                label = f"Box {index} -> ArUco {marker_id}"

                marker_center = tuple(
                    np.rint(markers[marker_id]).astype(int)
                )

                cv2.line(output, box_center, marker_center, color, 2)
            else:
                color = (0, 0, 255)
                label = f"Box {index} -> None"

            polygon = np.rint(box_points).astype(np.int32)
            cv2.polylines(output, [polygon], True, color, 2)
            cv2.circle(output, box_center, 4, color, -1)

            font = cv2.FONT_HERSHEY_SIMPLEX
            scale = 0.55
            thickness = 2

            (text_width, text_height), baseline = cv2.getTextSize(
                label, font, scale, thickness
            )

            text_x = max(0, min(int(x1), output.shape[1] - text_width - 8))
            text_y = max(text_height + 8, int(y1) - 10)

            cv2.rectangle(
                output,
                (text_x, text_y - text_height - 5),
                (text_x + text_width + 8, text_y + baseline + 4),
                (0, 0, 0),
                -1,
            )
            cv2.putText(
                output,
                label,
                (text_x + 4, text_y),
                font,
                scale,
                color,
                thickness,
                cv2.LINE_AA,
            )

            matches.append({"box": index, "marker_id": marker_id, "xyxy": [float(x1), float(y1), float(x2), float(y2)]})

        return output, {"markers": markers, "matches": matches}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('image', nargs='?', type=Path, default=ROOT / 'YOLO/assets/test_match_1.png')
    parser.add_argument('--weights', type=Path, default=DEFAULT_WEIGHTS)
    parser.add_argument('--device', default='cpu')
    parser.add_argument('--dictionary', default='DICT_4X4_250')
    parser.add_argument('--output', type=Path, default=Path('matched_result.jpg'))
    parser.add_argument('--headless', action='store_true')
    args = parser.parse_args()
    img = cv2.imread(str(args.image))
    if img is None:
        parser.error(f'Не удалось прочитать {args.image}')
    processor = FrameProcessor(args.weights, args.device, args.dictionary)
    output, detections = processor.process(img)
    for match in detections['matches']:
        print(f"Коробка {match['box']}: метка {match['marker_id']}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.output), output):
        raise RuntimeError(f'Не удалось сохранить {args.output}')
    if not args.headless:
        cv2.imshow('Boxes and ArUco', output)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
