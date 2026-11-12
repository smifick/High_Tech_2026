from pathlib import Path
import tempfile
import unittest
import json
from box_detector import MATCH_ANGLE_DEG
from scan_two_drones import Inventory, DEFAULT_MAP
from shared.match_box import match_box_to_marker


class InventoryTests(unittest.TestCase):
    def test_marker_above_box(self):
        box = [[10,10],[30,10],[30,30],[10,30]]
        markers = {100:[0,20],101:[20,40],102:[20,0],103:[40,20]}
        self.assertEqual(match_box_to_marker(box, markers, angle_deg=0), 100)
        self.assertEqual(match_box_to_marker(box, markers, angle_deg=MATCH_ANGLE_DEG), 102)
        self.assertIsNone(match_box_to_marker(box, {101: [20,40]}, angle_deg=MATCH_ANGLE_DEG))
        rectangle = [[10,20],[50,20],[50,40],[10,40]]
        self.assertEqual(match_box_to_marker(rectangle, {105: [30,10]}, angle_deg=MATCH_ANGLE_DEG), 105)

    def test_both_classes_and_coverage(self):
        inv = Inventory(DEFAULT_MAP)
        self.assertEqual(inv.expected, set(range(100,145)))
        inv.update('drone1', {'markers':{100:[],101:[],102:[],999:[]}, 'detections':[
            {'marker_id':101,'anomaly':False}, {'marker_id':102,'anomaly':True}]})
        inv.update('drone2', {'markers':{101:[],102:[],103:[]}, 'detections':[]})
        self.assertEqual(inv.states,{100:'пусто',101:'занято',102:'аномалия',103:'пусто'})
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'addresses.txt'
            inv.save(path)
            self.assertEqual(path.read_text(), '100: пусто\n101: занято\n102: аномалия\n103: пусто\n')
            report=json.loads(path.with_suffix('.coverage.json').read_text())
            self.assertEqual(report['unread_ids'],list(range(104,145)))
            self.assertEqual(report['unknown_ids'],[999])
            self.assertFalse(report['complete'])
            self.assertEqual(report['seen_by']['101'],['drone1','drone2'])

    def test_single_defect_can_be_corrected(self):
        inv=Inventory(DEFAULT_MAP)
        def observation(anomaly, confidence):
            return {'markers':{100:[]},'detections':[{'marker_id':100,'anomaly':anomaly,'confidence':confidence}]}
        inv.update('drone1',observation(True,.9))
        inv.update('drone2',observation(False,.8))
        inv.update('drone2',observation(False,.8))
        self.assertEqual(inv.states[100],'занято')
        for _ in range(9): inv.update('drone1',observation(True,.8))
        self.assertEqual(inv.states[100],'аномалия')
        self.assertEqual(len(inv.votes[100]),9)

    def test_unmatched_detection_is_reported(self):
        inv=Inventory(DEFAULT_MAP)
        box={'marker_id':None,'anomaly':True,'confidence':.8}
        inv.update('drone1',{'markers':{},'detections':[box]})
        self.assertEqual(inv.unmatched['drone1'],[box])
        self.assertEqual(inv.states,{})

    def test_unseen_marker_not_marked_empty(self):
        inv=Inventory(DEFAULT_MAP)
        inv.update('drone1',{'markers':{},'detections':[{'marker_id':None,'anomaly':True}]})
        self.assertEqual(inv.states,{})

if __name__=='__main__':
    unittest.main()
