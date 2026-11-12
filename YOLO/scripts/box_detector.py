"""Two-class box/defect detection for the live camera receiver."""
from pathlib import Path
import sys
import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from shared.match_box import match_box_to_marker
from ultralytics import YOLO

# Existing helper rotates into box-local axes; +90 selects a marker ABOVE the box.
MATCH_ANGLE_DEG = 90

DEFAULT_BOX_WEIGHTS = Path(__file__).resolve().with_name('best_for_classificator(2).pt')


def marker_overlapping_box(xyxy, marker_polygons, min_box_coverage=0.5):
    """Reject detections whose area is mostly an actual decoded ArUco polygon."""
    x1, y1, x2, y2 = xyxy
    area = (x2 - x1) * (y2 - y1)
    if area <= 0:
        return None
    rectangle = np.array([[x1,y1], [x2,y1], [x2,y2], [x1,y2]], dtype=np.float32)
    for marker_id, polygon in marker_polygons.items():
        hull = cv2.convexHull(np.asarray(polygon, dtype=np.float32).reshape(4, 2))
        intersection, _ = cv2.intersectConvexConvex(rectangle, hull)
        if intersection / area >= min_box_coverage:
            return marker_id
    return None


class BoxProcessor:
    def __init__(self, weights=DEFAULT_BOX_WEIGHTS, device='cpu', conf=.25, dictionary='DICT_4X4_250', match_angle=MATCH_ANGLE_DEG):
        if not Path(weights).is_file():
            raise FileNotFoundError(f'Веса не найдены: {weights}')
        self.model = YOLO(str(weights))
        if self.model.task != 'detect':
            raise ValueError('Нужна модель детекции объектов (detect)')
        if not dictionary.startswith('DICT_') or not hasattr(cv2.aruco, dictionary):
            raise ValueError(f'Неизвестный словарь ArUco: {dictionary}')
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary)),
            cv2.aruco.DetectorParameters())
        self.match_angle = match_angle
        self.device = device
        self.conf = conf

    def process(self, frame, associate_markers=True):
        result = self.model.predict(frame, device=self.device, conf=self.conf, imgsz=640, agnostic_nms=True, verbose=False)[0]
        corners, ids, _ = self.detector.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        markers = {} if ids is None else {
            int(marker_id): corner.reshape(4, 2).mean(axis=0).tolist()
            for marker_id, corner in zip(ids.flatten(), corners)
        }
        marker_polygons = {} if ids is None else {
            int(marker_id): corner.reshape(4, 2)
            for marker_id, corner in zip(ids.flatten(), corners)
        }
        output = frame.copy()
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(output, corners, ids)
        detections = []
        ignored = []
        counts = {name: 0 for name in result.names.values()}
        for xyxy, confidence, class_id in zip(result.boxes.xyxy.cpu().tolist(),
                                              result.boxes.conf.cpu().tolist(),
                                              result.boxes.cls.cpu().tolist()):
            class_id = int(class_id)
            name = result.names[class_id]
            false_marker = marker_overlapping_box(xyxy, marker_polygons)
            if false_marker is not None:
                ignored.append({'class_name': name, 'confidence': confidence, 'xyxy': xyxy,
                                'aruco_id': false_marker, 'reason': 'aruco_as_box'})
                continue
            counts[name] += 1
            x1, y1, x2, y2 = xyxy
            marker_id = match_box_to_marker(
                [[x1, y1], [x2, y1], [x2, y2], [x1, y2]], markers, angle_deg=self.match_angle)
            if not associate_markers:
                marker_id = None
            color = (0, 0, 255) if name == 'box_hitech_defect' else (0, 200, 0)
            cv2.rectangle(output, (round(x1), round(y1)), (round(x2), round(y2)), color, 2)
            label = f'{name} {confidence:.2f} -> ArUco {marker_id if marker_id is not None else "None"}'
            if not associate_markers:
                label = f'{name} {confidence:.2f}'
            (width, height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, .5, 1)
            tx = max(0, min(round(x1), output.shape[1] - width - 4))
            ty = max(height + 4, min(round(y1) - 8, output.shape[0] - baseline - 4))
            cv2.rectangle(output, (tx, ty-height-3), (tx+width+4, ty+baseline+3), (0, 0, 0), -1)
            cv2.putText(output, label, (tx+2, ty), cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1, cv2.LINE_AA)
            if marker_id is not None:
                center = (round((x1+x2)/2), round((y1+y2)/2))
                marker_center = tuple(np.rint(markers[marker_id]).astype(int))
                cv2.line(output, center, marker_center, color, 2)
            detections.append({'marker_id': marker_id, 'class_id' : class_id, 'class_name': name,
                               'confidence': confidence, 'xyxy': xyxy,
                               'anomaly': name == 'box_hitech_defect'})
        return output, {'markers': markers, 'detections': detections, 'counts': counts, 'ignored_detections': ignored}
