# Полёты на дроне

> Важно: python код выполняется на sverk obric

## Загрузка сетки:
Файл карты `aruco_nav` необходимо перенести в папку `~/sverk_ws/src/sverk_drone/odometry/aruco/aruco_map/config`, далее 
```bash
sudo systemctl restart sverk-ros2-docker.service
```

## Запуск полёта по аруко-сетке:
```bash
python3 flight_aruco.py
```

## Web socket client
Описан класс взаимодействия с внешним вычислителем через протокол web socket

# Подготовка датасета

В этой папке код для сбора датасета с дрона

> Важно: python код выполняется на sverk obric 


## Одиночный снимок с камеры дрона:
```bash
python3 take_pictures.py
```
сохраняет jpg снимок в папку photos

## Постоянные снимки с камеры дрона каждую секунду
```bash
python3 continuous_take_pictures.py
```
Делает то же самое, но каждую секунду

## Сохранение фотографий с дрона
```bash
scp -r sverk@<ip>:~/phtos ./path_on_your_pc
```

## Пример фото:
![example.jpg](prepare_dataset/assets/example.jpg)
