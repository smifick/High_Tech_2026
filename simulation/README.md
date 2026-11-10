# Симулятор с прототипом траектории обхода 4 дронами поля 10x10

## Структура
```text
simulation
├── logic
├── scene
└── scripts
```

## scripts
Здесь находятся скрипты запуска симулятора

### Генерация поля
```bash
PYTHONPATH=. python ./simulation/scripts/generate_field.py --drones 4 --boxes 15 --seed 42
```

### Выставление дронов по углам и получение их базовых координат относительно сетки:
```bash
python base_xy.py
```

### Запуск обхода дронами по сетке с базовым позиционированием:
```bash
python scan_field.py
```

## logic
```text
logic
├── grid       # логика работы с сеткой
├── simulation # симуляция дрона
└── vision     # работа с ArUco
```

## scene
Текстуры ArUco и xml разметка для mujoco
