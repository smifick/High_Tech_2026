# YOLO
В этом модуле код, связанный с обучением YOLO и работы с CV

## Базовый YOLO для инвентаризации коробок

Прототип обработки сохранённых кадров для кейса с дронами и ровером.
YOLO11n определяет коробку и видимые аномалии; ArUco задаёт постоянный адрес места.


Установка и запуск:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
# Укажите путь к данным в data.yaml
python train.py --device 0 --epochs 100
# Без CUDA: --device cpu (существенно медленнее)
python infer.py --weights runs/boxes/weights/best_drone_photos_yolo.pt \
  --source frames/drone1 --attempt attempts/001 --agent drone-1
python infer.py --weights runs/boxes/weights/best_drone_photos_yolo.pt \
  --source frames/drone2 --attempt attempts/001 --agent drone-2
python infer.py --weights runs/boxes/weights/best_drone_photos_yolo.pt \
  --source frames/rover/place24 --attempt attempts/001 \
  --view side --place-id 24 --agent rover
python -m unittest discover -s tests -v
```
## Данные и разметка

```text
dataset/
  images/train/   images/val/   images/test/
  labels/train/   labels/val/   labels/test/
```

Одинаковые имена пар, например `images/train/001.jpg` и `labels/train/001.txt`.
Одна строка YOLO: `class_id center_x center_y width height`, координаты 0–1.

| ID | Класс | Как размечать |
|---|---|---|
| 0 | box_ok | Вся исправная коробка |
| 1 | box_damaged | Вся коробка с видимым повреждением |
| 2 | foreign_object | Посторонний предмет отдельной рамкой |

Проверка детектора после обучения:

```bash
yolo detect val model=runs/boxes/weights/best_drone_photos_yolo.pt data=data.yaml split=test imgsz=960
```

Отдельно измеряйте пропуски аномалий, ложные отметки пустоты и точность
привязки к месту. mAP детектора не оценивает всю ведомость.

## Обязательная настройка площадки

## Решения и файлы

- `occupied`: одна уверенная исправная коробка.
- `anomaly`: уверенное повреждение или посторонний предмет.
- `empty`: разрешённая эвристика пустоты после нескольких наблюдений.
- `unknown`: неуверенность, конфликт кадров, несколько обычных коробок,
  ROI выходит за кадр либо нет детекций при запрещённой эвристике пустоты.
- `unseen`: ожидаемое место не встречалось.

Для консервативного старта все места кроме подтверждённой пустоты идут роверу:
включая `occupied`, поскольку сверху не видны повреждения боковин.
Уверенное решение ровера окончательное; последнее по порядку добавления
заменяет предыдущее. Отсутствие детекции у ровера остаётся `unknown`.
Снимок ровера должен содержать только целевое место: `--place-id` поступает
от навигации, код не подтверждает эту привязку визуально.

В общей папке попытки:

- `observations.jsonl`: все наблюдения, рамки в координатах ROI, путь к кадру,
  аппарат и время обработки (не время съёмки).
- `inventory.csv`: одна строка на адрес; статус, источник, очередь проверки.
- `rover_queue.json`: ID мест для последующей проверки.

Храните исходные кадры вместе с журналом для проверки ошибок. Между вылетами
состояние площадки должно быть неизменным. Если груз переставили, начните
новую попытку. Скрипт объединяет результаты аппаратов, но не координирует их.
Предполагается известный набор типов аномалий: незнакомые повреждения YOLO
может пропустить. Для них позже можно добавить отдельный anomaly detector.

Ultralytics распространяется на условиях AGPL-3.0/коммерческой лицензии;
проверьте подходящий вариант перед включением в закрытый продукт.

## Структура проекта

| Файл | Назначение |
|---|---|
| `train.py` | Обучение трёхклассового YOLO11n |
| `infer.py` | Обработка кадров и формирование ведомости |
| `logic.py` | Определение состояния и объединение наблюдений |
| `config.json` | ArUco, область места и пороги решений |
| `data.yaml` | Пути датасета и классы |
| `tests` | Тесты логики решений |

## Приложения
Результаты обучения
F1-Confidence Curve
![Результаты обучения](YOLO/YOLO11n_models/YOLO11n_baseline/train/training_info/BoxF1_curve.png)
Распознавание объектов
![Результаты обучения](YOLO/YOLO11n_models/YOLO11n_baseline/train/training_info/val_batch0_labels.jpg)

