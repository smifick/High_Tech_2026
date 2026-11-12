import unittest
from types import SimpleNamespace
import numpy as np
from box_detector import BoxProcessor, marker_overlapping_box
from scan_two_drones import Inventory, DEFAULT_MAP


def tensor(values):
    return SimpleNamespace(cpu=lambda: np.asarray(values))


class MarkerFilterTests(unittest.TestCase):
    def test_exact_padded_and_partial_marker(self):
        markers={100:np.array([[10,10],[30,10],[30,30],[10,30]])}
        for box in ([10,10,30,30],[6,6,34,34],[12,12,28,28]):
            self.assertEqual(marker_overlapping_box(box,markers),100)
        self.assertIsNone(marker_overlapping_box([10,28,30,48],markers))
        self.assertIsNone(marker_overlapping_box([0,0,100,100],markers))
        self.assertIsNone(marker_overlapping_box([10,10,30,30],{}))

    def test_rotated_polygon_not_bounding_rectangle(self):
        markers={101:np.array([[20,0],[40,20],[20,40],[0,20]])}
        self.assertIsNone(marker_overlapping_box([0,0,8,8],markers))
        self.assertEqual(marker_overlapping_box([10,10,30,30],markers),101)

    def test_both_classes_filtered_before_counts_and_inventory(self):
        for false_class in (0,1):
            processor=BoxProcessor.__new__(BoxProcessor)
            processor.device='cpu'; processor.conf=.25
            result=SimpleNamespace(names={0:'box_hitech',1:'box_hitech_defect'},
                boxes=SimpleNamespace(xyxy=tensor([[10,10,30,30],[10,32,30,52]]),
                    conf=tensor([.9,.8]), cls=tensor([false_class,1])))
            processor.model=SimpleNamespace(predict=lambda *args,**kwargs:[result])
            processor.detector=SimpleNamespace(detectMarkers=lambda _: (
                [np.array([[[10,10],[30,10],[30,30],[10,30]]],dtype=np.float32)],np.array([[100]]),[]))
            _,data=processor.process(np.zeros((80,80,3),dtype=np.uint8))
            self.assertEqual(len(data['detections']),1)
            self.assertEqual(data['detections'][0]['marker_id'],100)
            self.assertEqual(data['counts'],{'box_hitech':0,'box_hitech_defect':1})
            self.assertEqual(data['ignored_detections'][0]['aruco_id'],100)
            inv=Inventory(DEFAULT_MAP);inv.update('drone1',data)
            self.assertEqual(inv.states,{100:'аномалия'})

if __name__=='__main__': unittest.main()
