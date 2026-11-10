# Полёты на дроне

> Важно: python код выполняется на sverk obric

## Загрузка сетки:
Файл карты `aruco_map_750x450_id100.txt` необходимо перенести в папку `~/sverk_ws/src/sverk_drone/odometry/aruco/aruco_map/config`, далее 
```bash
sudo systemctl restart sverk-ros2-docker.service
```

## Запуск полёта по аруко-сетке:
```bash
python3 flight_aruco.py
```
