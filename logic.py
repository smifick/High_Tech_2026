"""Pure decision logic; no model dependencies."""
CLASSES = {0: 'box_ok', 1: 'box_damaged', 2: 'foreign_object'}


def classify(detections, threshold, allow_empty=False):
    if not detections:
        return 'empty' if allow_empty else 'unknown'
    if any(d['confidence'] < threshold for d in detections):
        return 'unknown'
    if any(d['class'] in ('box_damaged', 'foreign_object') for d in detections):
        return 'anomaly'
    return 'occupied' if len(detections) == 1 else 'unknown'


def summarize(observations, expected_ids=(), empty_observations=3):
    rows = []
    ids = sorted(set(expected_ids) | {o['place_id'] for o in observations})
    for place_id in ids:
        history = [o for o in observations if o['place_id'] == place_id]
        # append order is acquisition order within one attempt
        rover = [o for o in history if o['view'] == 'side' and o['status'] != 'unknown']
        source = 'none'
        status = 'unseen'
        if rover:
            status, source = rover[-1]['status'], 'rover'
        elif history:
            top = [o for o in history if o['view'] == 'top']
            statuses = {o['status'] for o in top}
            status, source = 'unknown', 'drone'
            if len(statuses) == 1:
                candidate = next(iter(statuses))
                if candidate != 'empty' or len(top) >= empty_observations:
                    status = candidate
        rows.append({'place_id': place_id, 'status': status, 'source': source,
                     'needs_rover': source != 'rover' and status != 'empty',
                     'observations': len(history)})
    return rows
