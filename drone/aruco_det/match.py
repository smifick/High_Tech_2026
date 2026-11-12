"""Compatibility detector; live inventory is launched with python -m station."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'YOLO/scripts'))
from box_detector import BoxProcessor, DEFAULT_BOX_WEIGHTS

_processor = None
_processor_path = None


def scan(img, path_to_model=DEFAULT_BOX_WEIGHTS):
    """Return complete detections including ID, class and confidence; cache weights."""
    global _processor, _processor_path
    model_path = str(Path(path_to_model).resolve())
    if _processor is None or model_path != _processor_path:
        _processor = BoxProcessor(model_path, dictionary='DICT_4X4_1000')
        _processor_path = model_path
    _, result = _processor.process(img)
    return [dict(box, box_center=(round((box['xyxy'][0] + box['xyxy'][2]) / 2),
                                       round((box['xyxy'][1] + box['xyxy'][3]) / 2)))
            for box in result['detections']]


if __name__ == '__main__':
    import sverk_interfaces
    from time import sleep
    drone = sverk_interfaces.init(Nodename='fly_scan')
    try:
        while True:
            print(scan(drone.image.take_picture(timeout=5.0)))
            sleep(1)
    finally:
        drone.close()
