# Computer Vision & Control for Autonomous Mobile Platform

**Computer Vision / Robotics проект автономной мобильной платформы на NVIDIA Jetson Xavier.**

Камера ZED передаёт изображение, CV-модель обнаруживает конусы трассы, система строит центральную траекторию и целевую точку, после чего рассчитывает управляющее воздействие и передаёт команды на Arduino.

> Основной фокус проекта: real-time Computer Vision на edge-устройстве - YOLO, TensorRT/CUDA, многопоточный pipeline, построение траектории и управление мобильной платформой.

## Что делает проект

1. NVIDIA Jetson Xavier получает кадры с ZED Camera.
2. TensorRT-модель обнаруживает конусы и определяет их классы.
3. Детекции переводятся в координаты относительно камеры.
4. По левым и правым границам трассы строится centerline.
5. Выбирается lookahead-точка и сглаживается целевое направление.
6. PID-регулятор рассчитывает steering.
7. Команды движения передаются на Arduino.
8. С отдельного ПК доступны ручной/автоматический режим, изменение скорости, запись и телеметрия.

## Моя роль - Computer Vision / Robotics Engineer

Проект для СКБ УУНиТ, команда из двух человек. Я отвечал за **Computer Vision, оптимизацию inference и алгоритм навигации**:

- подготовил и размечал датасет и обучил **YOLO-модель для детекции конусов**, используемых системой навигации;
- конвертировал модель в **TensorRT Engine** и оптимизировал её запуск на **NVIDIA Jetson Xavier**;
- увеличил производительность обработки видео примерно с **3–4 FPS до 50–60 FPS** при сопоставимых условиях запуска;
- разделил получение кадров, inference и обработку результатов между независимыми потоками;
- использовал очереди ограниченного размера и обработку только части входящих кадров, чтобы система не накапливала устаревшие данные и управляла платформой по максимально свежему изображению;
- разработал алгоритм построения траектории по результатам Computer Vision и данным **ZED Camera**;
- участвовал в проверке алгоритма в симуляции **Unreal Engine 4**; дальнейшее развитие проекта связано с переходом на **ROS 2**.

## Архитектура

```text
ZED Camera
    │
    ▼
Frame capture
    │
    ├──────────────► Web / Recording
    │
    ▼
Frame decimation
    │
    ▼
TensorRT YOLO
    │
    ▼
Cone detections
    │
    ▼
Coordinate estimation
    │
    ▼
Track boundaries
    │
    ▼
Centerline + Lookahead
    │
    ▼
PID steering
    │
    ▼
Arduino / Motors
```

Отдельный ПК взаимодействует с Jetson по UDP и позволяет переключать manual/auto mode, менять скорость и управлять записью.

## Оптимизация real-time pipeline

Пайплайн использует:

- очередь ограниченного размера кадра, чтобы система не увеличивала задержку
- обработку не каждого кадра через `process_every`;
- уменьшенное изображение для детектора;
- OpenCV CUDA при доступности;
- TensorRT + PyCUDA;
- отдельный CUDA context для рабочего inference-потока;
- асинхронный post-processing результатов;
- отдельную очередь записи видео.

## Построение траектории

После детекции система:

- разделяет конусы по классам/сторонам трассы;
- оценивает их положение;
- интерполирует левую и правую границы;
- строит centerline;
- выбирает точку впереди автомобиля по lookahead;
- сглаживает целевую точку через EMA;
- рассчитывает скорость через PID-регулятор.


## Технологии

**Computer Vision / ML**
- Python
- PyTorch / YOLO
- OpenCV
- TensorRT
- CUDA / PyCUDA
- NumPy

**Robotics / Edge**
- NVIDIA Jetson Xavier
- ZED Camera / ZED SDK
- Arduino
- Serial
- UDP
- PID control
- multithreading

**Дополнительно**
- Flask для web-stream
- FFmpeg / video recording
- Unreal Engine 4 для симуляции
- ROS 2 — направление дальнейшей интеграции

## Структура репозитория

```text
.
├── Jetson Xavier/
│   ├── server.py                  # Основной real-time pipeline
│   ├── config.jsonc
│   └── Code/
│       ├── Cone_detector.py       # TensorRT inference
│       ├── Car_control.py         # Arduino / управление
│       ├── Config_load.py
│       └── Web.py                 # Web-stream
│
├── PC/
│   ├── server_send.py             # Пульт управления
│   └── arduino_example_move.py
│
├── Yolo/                          # Эксперименты с YOLO
└── Задачи проекта.md
```

## Запуск проекта

Для полного запуска нужны:

- NVIDIA Jetson Xavier;
- ZED Camera;
- Arduino и мобильная платформа;
- TensorRT engine обученного детектора конусов;
- совместимые версии JetPack, CUDA, TensorRT и ZED SDK.

Запуск на Jetson:

```bash
cd "Jetson Xavier"
python3 server.py
```

Запуск пульта на отдельном ПК:

```bash
python PC/server_send.py
```

