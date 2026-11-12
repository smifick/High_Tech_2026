"""Reproducible per-frame report; never invents ground truth or field accuracy."""
import argparse
import csv
import hashlib
import html
import json
from pathlib import Path
import time

import cv2
from box_detector import BoxProcessor, DEFAULT_BOX_WEIGHTS


def iou(a, b):
    width = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    height = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    intersection = width * height
    union = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - intersection
    return intersection / union if union > 0 else 0


def score(detections, truth):
    # Greedy one-to-one, confidence-ranked, same-class matching at IoU >= 0.5.
    used = set()
    tp = 0
    for detection in sorted(detections, key=lambda d: d['confidence'], reverse=True):
        candidates = [(iou(detection['xyxy'], box['xyxy']), index)
                      for index, box in enumerate(truth)
                      if index not in used and box['class_name'] == detection['class_name']]
        overlap, index = max(candidates, default=(0, -1))
        if overlap >= .5:
            used.add(index)
            tp += 1
    return {'tp': tp, 'fp': len(detections)-tp, 'fn': len(truth)-tp}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--images', nargs='+', type=Path, required=True)
    parser.add_argument('--weights', type=Path, default=DEFAULT_BOX_WEIGHTS)
    parser.add_argument('--labels', type=Path, help='JSON: image filename -> list of {class_name, xyxy}; empty list is explicit negative')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--match-angle', type=int, choices=(0,90,180,270), default=90)
    args = parser.parse_args()
    truth = json.loads(args.labels.read_text()) if args.labels else {}
    args.output.mkdir(parents=True, exist_ok=True)
    processor = BoxProcessor(args.weights, dictionary='DICT_4X4_1000', match_angle=args.match_angle)
    rows = []
    cards = []
    totals = {'tp': 0, 'fp': 0, 'fn': 0}
    labeled = 0
    for index, image in enumerate(args.images):
        frame = cv2.imread(str(image))
        if frame is None:
            raise ValueError(f'Не удалось прочитать {image}')
        started = time.perf_counter()
        annotated, result = processor.process(frame)
        elapsed = time.perf_counter() - started
        annotated_name = f'{index:03d}_detected.jpg'
        raw_name = f'{index:03d}_raw.jpg'
        if not cv2.imwrite(str(args.output/annotated_name), annotated) or not cv2.imwrite(str(args.output/raw_name), frame):
            raise OSError('Не удалось сохранить кадр')
        metrics = score(result['detections'], truth[image.name]) if image.name in truth else None
        if metrics is not None:
            labeled += 1
            for key in totals:
                totals[key] += metrics[key]
        row = {'image': image.as_posix(), 'sha256': hashlib.sha256(image.read_bytes()).hexdigest(),
               'detections': len(result['detections']), 'matched': sum(d['marker_id'] is not None for d in result['detections']),
               'seconds': round(elapsed, 3), 'metrics_iou50': metrics, 'result': result}
        rows.append(row)
        cards.append(f'<h2>{html.escape(image.name)}</h2><p>Найдено: {row["detections"]}; с ID: {row["matched"]}; {elapsed:.2f} с. '
                     f'{html.escape(str(metrics)) if metrics is not None else "Ручной разметки нет: точность не вычислялась."}</p>'
                     f'<img src="{raw_name}"><img src="{annotated_name}">')
    summary = {'weights': str(args.weights), 'weights_sha256': hashlib.sha256(args.weights.read_bytes()).hexdigest(),
               'dictionary': 'DICT_4X4_1000', 'match_angle': args.match_angle, 'confidence': .25, 'imgsz': 640,
               'frames': len(rows), 'labeled_frames': labeled, 'totals_iou50': totals if labeled else None,
               'precision': totals['tp']/(totals['tp']+totals['fp']) if totals['tp']+totals['fp'] else None,
               'recall': totals['tp']/(totals['tp']+totals['fn']) if totals['tp']+totals['fn'] else None,
               'rows': rows}
    (args.output/'report.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    with (args.output/'frames.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['image','detections','matched','seconds','tp','fp','fn'])
        for row in rows:
            writer.writerow([row['image'],row['detections'],row['matched'],row['seconds'],
                             *[row['metrics_iou50'][key] if row['metrics_iou50'] else '' for key in ('tp','fp','fn')]])
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(9,4))
    ax.bar(range(len(rows)), [row['detections'] for row in rows], label='Detected boxes')
    ax.bar(range(len(rows)), [row['matched'] for row in rows], label='Matched to ArUco', alpha=.7)
    ax.set(xlabel='Frame index', ylabel='Count', title='Detections and ArUco matches (not accuracy)')
    ax.legend()
    fig.tight_layout()
    fig.savefig(args.output/'comparison.png')
    plt.close(fig)
    (args.output/'index.html').write_text('<!doctype html><html lang="ru"><meta charset="utf-8"><title>Отчёт распознавания</title>'
        '<style>body{font:16px system-ui;max-width:1200px;margin:30px auto}img{width:48%;vertical-align:top}</style>'
        '<h1>Покадровое распознавание</h1><p>Исходник слева, результат справа. '
        'Кадры взяты из репозитория; независимость от обучения и происхождение с конкурсной площадки не подтверждены. '
        'Количество найденных объектов не является оценкой точности. Для P/R нужна независимая ручная разметка. '
        'Метод при разметке: confidence-ranked matching, одинаковый класс, IoU ≥ 0.5; это не mAP.</p>'
        '<p><a href="report.json">Данные и SHA256</a> · <a href="frames.csv">CSV</a></p><img src="comparison.png">'+''.join(cards))


if __name__ == '__main__':
    main()
