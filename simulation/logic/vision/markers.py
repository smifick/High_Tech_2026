"""Чтение ArUco-маркеров с камеры дрона."""
import cv2

WIDTH = 640
HEIGHT = 480

def get_markers(drone):
    image = drone.get_image(width=WIDTH, height=HEIGHT)

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    detector = cv2.aruco.ArucoDetector(dictionary)

    corners, ids, rejected = detector.detectMarkers(image)

    if ids is None:
        return [], [], rejected

    centers = [corner.reshape(4, 2).mean(axis=0).tolist() for corner in corners]

    return centers, ids.flatten().tolist(), rejected


def nearest_marker(drone):
    centers, ids, _ = get_markers(drone)
    if not centers:
        return None
    index = min(range(len(centers)), key=lambda i:
                (centers[i][0] - WIDTH / 2) ** 2 + (centers[i][1] - HEIGHT / 2) ** 2)
    return ids[index]
