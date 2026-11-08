import unittest
from logic import classify, summarize


def obs(status, view='top', place=24):
    return dict(place_id=place, status=status, view=view)


class Decisions(unittest.TestCase):
    def test_no_detection_is_unknown_by_default(self):
        self.assertEqual(classify([], .65), 'unknown')

    def test_damage_and_foreign_object(self):
        for name in ['box_damaged', 'foreign_object']:
            self.assertEqual(classify([dict(class_=name, **{'class': name}, confidence=.9)], .65), 'anomaly')

    def test_low_confidence_and_multiple_boxes(self):
        d = {'class': 'box_ok', 'confidence': .4}
        self.assertEqual(classify([d], .65), 'unknown')
        d['confidence'] = .9
        self.assertEqual(classify([d, d], .65), 'unknown')

    def test_rover_wins_over_later_drone(self):
        row = summarize([obs('anomaly'), obs('occupied', 'side'), obs('anomaly')])[0]
        self.assertEqual(row['status'], 'occupied')
        self.assertFalse(row['needs_rover'])

    def test_uncertain_rover_does_not_erase_decision(self):
        row = summarize([obs('anomaly', 'side'), obs('unknown', 'side')])[0]
        self.assertEqual(row['status'], 'anomaly')

    def test_empty_requires_repeated_observation(self):
        self.assertEqual(summarize([obs('empty')])[0]['status'], 'unknown')
        self.assertEqual(summarize([obs('empty')] * 3)[0]['status'], 'empty')

    def test_conflict_goes_to_rover(self):
        row = summarize([obs('occupied'), obs('empty')])[0]
        self.assertEqual(row['status'], 'unknown')
        self.assertTrue(row['needs_rover'])

    def test_expected_places_and_no_duplicate_rows(self):
        rows = summarize([obs('occupied')] * 3, [24, 25])
        self.assertEqual(len(rows), 2)
        self.assertEqual(rows[1]['status'], 'unseen')
        self.assertTrue(rows[1]['needs_rover'])


if __name__ == '__main__':
    unittest.main()
