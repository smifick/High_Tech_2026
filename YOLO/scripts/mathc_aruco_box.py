from pathlib import Path
import cv2
import numpy as np
from ultralytics import YOLO
from shared.match_box import match_box_to_marker


dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

img = cv2.imread(f"/data/Projects/High_Tech_2026/YOLO/assets/test_match_1.png")

if img is None:
    raise FileNotFoundError("Не удалось прочитать изображение")

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
corners, ids, rejected = detector.detectMarkers(gray)

markers = {}

if ids is not None:
    centers = [
        corner.reshape(4, 2).mean(axis=0).tolist()
        for corner in corners
    ]
    markers = dict(zip(ids.flatten().tolist(), centers))

folder = Path(__file__).resolve().parent
model = YOLO('/data/Projects/High_Tech_2026/YOLO/YOLO11n_models/YOLO11n_finetune/best_for_finetune.pt')
result = model.predict(
    source=img,
    device="cpu",
    verbose=False,
)[0]

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

    print(f"Коробка {index}: метка {marker_id}")

output_path = "/data/Projects/High_Tech_2026/YOLO/assets/matched_result.jpg"

if not cv2.imwrite(output_path, output):
    raise RuntimeError(f"Не удалось сохранить {output_path}")

cv2.namedWindow("Boxes and ArUco", cv2.WINDOW_NORMAL)
cv2.imshow("Boxes and ArUco", output)
cv2.waitKey(0)
cv2.destroyAllWindows()