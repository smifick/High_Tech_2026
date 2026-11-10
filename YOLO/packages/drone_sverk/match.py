import cv2
from ultralytics import YOLO
from packages.match_box import match_box_to_marker
import sverk_interfaces
from time import sleep

def scan(img, path_to_model):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray)

    markers = {}

    if ids is not None:
        centers = [
            corner.reshape(4, 2).mean(axis=0).tolist()
            for corner in corners
        ]
        markers = dict(zip(ids.flatten().tolist(), centers))

    model = YOLO(str(path_to_model))
    result = model.predict(
        source=img,
        device="cpu",
        verbose=False,
    )[0]

    res = []

    for index, (x1, y1, x2, y2) in enumerate(result.boxes.xyxy.cpu().numpy(), start=1):
        box_points = [
            [float(x1), float(y1)],
            [float(x2), float(y1)],
            [float(x2), float(y2)],
            [float(x1), float(y2)],
        ]

        marker_id = match_box_to_marker(box_points, markers, angle_deg=0)

        box_center = (
            int(round((x1 + x2) / 2)),
            int(round((y1 + y2) / 2)),
        )

        res.append({
            "marker_id": marker_id,
            "box_center": box_center,
        })

    return res

if __name__ == "__main__":
    drone = sverk_interfaces.init(Nodename="fly_scan")

    while True:
        image = drone.image.take_picture(timeout=5.0)
        print(scan(image, './yolo.pt'))
        sleep(1)

