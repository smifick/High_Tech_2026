import cv2
from packages.match_box import match_box_to_marker


dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, parameters)

img = cv2.imread("image.jpg")

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

# TODO: get boxes via YOLO.
boxes = []

for box_points in boxes:
    marker_id = match_box_to_marker(box_points, markers, angle_deg=0)
    print(f"Коробка: {box_points}, метка: {marker_id}")