# car_control_project_new_Danis — simulation

Это ветка симуляции: общий controller core работает с лёгким Python-симулятором,
FSDS 2.2.0 через ROS2 Humble и аппаратным ROS1-контуром. Для FSDS добавлены
отдельные карты восьмёрки, змейки и поворотов, запись камеры и логи заездов.
Полная [инструкция установки](docs/SIMULATION_SETUP.md) и
[описание трасс и записи](docs/SIMULATION_TESTING.md).

## Что нужно установить

- Windows: Python 3 и готовый FSDS 2.2.0 в `C:\FSDS` (файл `C:\FSDS\FSDS.exe`).
- WSL2: Ubuntu 22.04, ROS2 Humble, `colcon` и зависимости из
  [инструкции](docs/SIMULATION_SETUP.md).
- Внешний checkout исходников FSDS v2.2.0 для сборки ROS2-моста. Команда ниже
  готовит только `AirSim` и `ros2`; Unreal Editor для управления штатной
  машиной и CSV-картами не требуется.

```bash
cd /home/danis/car_control_project_new_Danis
python3 tools/prepare_fsds.py --destination /home/danis/fsds-v2.2.0
bash tools/build_ros2.sh --fsds /home/danis/fsds-v2.2.0
```

Если FSDS или ROS2 уже установлены и workspace собран, повторная установка не нужна.

## Запуск тестовой карты

Закройте прежнее окно FSDS. В Windows PowerShell выполните:

```powershell
python "\\wsl.localhost\Ubuntu-22.04\home\danis\car_control_project_new_Danis\tools\launch_fsds_test.py" --track test_ground
```

Вместо `test_ground` доступны `figure_eight`, `slalom`, `turns`. Скрипт выводит
`host` для ROS2 и создаёт отдельный снимок карты и настроек в `C:\FSDS\test_runs`.
На Radeon Vega он применяет проверенный обход стартового сбоя Vulkan. По состоянию
на 19.09.2026 карта загрузилась, ROS2 получил 470 конусов, камера записала
изображение над дорогой и MP4. Длительная устойчивость и прохождение трасс
автопилотом пока не подтверждены.

В отдельном терминале WSL:

```bash
cd /home/danis/car_control_project_new_Danis
source /opt/ros/humble/setup.bash
source simulation_ws/install/setup.bash
ros2 launch car_control_sim fsds_drive.launch.py host:=АДРЕС_ИЗ_СКРИПТА
```

Панель управления запускается из Windows PowerShell:

```powershell
pythonw "\\wsl.localhost\Ubuntu-22.04\home\danis\car_control_project_new_Danis\tools\fsds_control_panel.py"
```

Профиль начинает в `MANUAL`. Кнопки `START RECORD`/`STOP RECORD` сохраняют
кадры, MP4, телеметрию и отчёт в `recordings/`. Для пользовательских моделей
SolidWorks понадобится отдельный импорт в полный Unreal-проект FSDS; сейчас
карты используют штатные модели машины и конусов. CAD-исходники не публикуются.

Один controller core для Jetson/ROS1, лёгкого симулятора и FSDS/ROS2.
Ветка создана от `ros`; обратно ничего не сливается.

**[Установка и запуск simulation → docs/SIMULATION_SETUP.md](docs/SIMULATION_SETUP.md)**

## Что изменено сегодня

- Добавлен общий controller core: его используют лёгкая симуляция, ROS2/FSDS и
  существующий Jetson/ROS1-контур; алгоритм управления не дублируется.
- FSDS закреплён как внешняя dependency v2.2.0 с официальным `fsds_ros2_bridge`;
  добавлены надёжная передача Track, watchdog команды, проверки диапазонов и
  корректные world/vehicle преобразования конусов.
- Добавлены два профиля: `fsds_jetson.launch.py` для сравнения с реальными
  настройками и `fsds_drive.launch.py` для удобного заезда в FSDS.
- В `fsds_drive` оранжевые стартовые конусы не останавливают машину, дальность
  поиска увеличена до 15 m, FOV до 140°, а ограничитель скорости стал плавным —
  без цикла «газ → полный тормоз → остановка».
- Добавлены MANUAL/AUTOPILOT/STOP, Windows-панель поверх FSDS и автоматический
  переход в MANUAL при нажатии стрелки во время автопилота.
- В панель добавлены runtime target до 15 m/s, предел газа, live status и запись
  штатной передней камеры вместе с CSV-телеметрией и итоговыми метриками заезда.
- Добавлен feedback-регулятор скорости по FSDS odometry с anti-windup; прежний
  limiter сохранён по умолчанию для сравнительного `fsds_jetson` профиля.
- Добавлены unit, ROS1, ROS2, lightweight и FSDS motion-smoke проверки.

## Быстрый запуск FSDS без тестовых карт

1. На Windows запустите FSDS и загрузите трассу.
2. В WSL откройте репозиторий и запустите профиль:

```bash
cd ~/car_control_project_new_Danis
source /opt/ros/humble/setup.bash
source simulation_ws/install/setup.bash

# Для данного ноутбука с WSL NAT:
FSDS_HOST=$(ip route show default | awk '{print $3; exit}')
ros2 launch car_control_sim fsds_drive.launch.py host:="$FSDS_HOST"
```

При включённом WSL mirrored networking вместо переменной можно указать
`host:=localhost`. Профиль начинает в `MANUAL`: сначала поставьте машину стрелками
примерно вдоль трассы, затем включите автопилот.

Панель управления из Windows PowerShell:

```powershell
pythonw "\\wsl.localhost\Ubuntu-22.04\home\danis\car_control_project_new_Danis\tools\fsds_control_panel.py"
```

Окно можно растягивать мышью за края и углы; ползунки автоматически занимают
доступную ширину. Минимальный размер оставляет доступными основные controls.

- `MANUAL` — стрелки управляют машиной.
- `AUTOPILOT` — shared controller ведёт машину по конусам.
- `STOP` — controller выключен, bridge отправляет тормоз.
- Стрелка во время `AUTOPILOT` автоматически возвращает `MANUAL`.
- `Turn sharpness` меняет steering `Kp`, `Turn response speed` — EMA response,
  а `Maximum steering command` ограничивает максимальную команду руля.

Без панели переключение выполняется из sourced WSL shell:

```bash
python3 tools/fsds_mode.py manual
python3 tools/fsds_mode.py auto
python3 tools/fsds_mode.py stop
```

## Скорость и `throttle_scale`

Скорость теперь можно менять во время заезда: в Windows-панели выберите
`Autopilot target (m/s)`, `Maximum FSDS throttle` и нажмите
`Apply speed + throttle`. Подтверждение появляется только после ответа controller.
Диапазон target 0.1–15.0 m/s; это диапазон ввода,
а не гарантия прохождения трассы на любой из этих скоростей. Изменение target
не включает автопилот и не меняет ручной режим. После перезапуска launch
возвращается значение из launch/его аргументов.

Из второго WSL-терминала после source окружения:

```bash
python3 tools/fsds_speed.py 2.0 --throttle-scale 0.20
```

Одновременная настройка поворота:

```bash
python3 tools/fsds_speed.py 2.5 --throttle-scale 0.20 \
  --steering-gain 0.80 --steering-response 0.50 --steering-limit 0.70
```

- `steering_gain`: выше — сильнее коррекция на ту же ошибку трассы.
- `steering_response`: выше — быстрее реакция и меньше сглаживание.
- `steering_limit`: максимальная команда руля в диапазоне 0..1.

Меняйте их небольшими шагами. Слишком большие gain/response вызывают рыскание,
а слишком маленькие — поздний вход в поворот.

После обновления кода пересоберите ROS2 и перезапустите launch и панель:
`bash tools/build_ros2.sh --fsds "$HOME/fsds-v2.2.0"`.

Текущие безопасно проверенные значения находятся в
`ros2/car_control_sim/launch/fsds_drive.launch.py`:

```text
fsds_max_speed_mps = 2.5
fsds_throttle_scale = 0.20
```

Их удобнее менять на запуске, не редактируя файл:

```bash
ros2 launch car_control_sim fsds_drive.launch.py host:="$FSDS_HOST" \
  fsds_max_speed_mps:=2.5 fsds_throttle_scale:=0.20
```

`fsds_max_speed_mps` — цель PI-регулятора скорости в м/с. Регулятор сравнивает её
с фактической скоростью из FSDS: при недоборе добавляет газ, при превышении
включает пропорциональный тормоз. Из-за инерции PhysX возможен короткий выбег.
`throttle_scale` ограничивает максимальный газ FSDS: при `0.20` регулятор никогда
не передаст больше `0.20`, даже когда shared controller запрашивает полный газ.

- Увеличение `throttle_scale` делает разгон сильнее и быстрее, но повышает выбег,
  риск колебаний скорости и схода в повороте.
- Уменьшение делает разгон мягче; слишком малое значение не преодолеет статическое
  трение виртуальной TechnionCar и машина не тронется.
- Сначала меняйте scale маленькими шагами `0.02`, затем проверяйте заезд:
  `python3 tests/fsds_smoke.py --seconds 15 --trace`.

Параметры геометрии, PID, дальности и цветов находятся в
`ros2/car_control_sim/config/fsds_drive.json`. Точный профиль текущей Jetson
машины отдельно сохранён в `fsds_jetson.launch.py`.

## Запись заезда

`START RECORD` в панели начинает запись передней камеры машины и телеметрии.
Это изображение штатной FSDS-камеры, а не запись экрана или вида оператора.
`STOP RECORD` корректно завершает MP4 и отчёт. Результат создаётся в
уникальной папке `recordings/YYYYMMDD_HHMMSS_ffffff/`:

- `camera.mp4` — вид с передней камеры;
- `frames/*.png` и `frames.csv` — исходные кадры и время их получения;
- `telemetry.csv` — положение, скорость, target, газ, руль, тормоз и возраст данных;
- `events.jsonl` — статусы, команды, конусы, параметры и ошибки ROS;
- `summary.json` и `report.md` — показатели и предупреждения заезда;
- `recorder.log` — диагностика записи.

При закрытии панели текущая запись завершается, controller отключается, а FSDS
возвращает управление стрелкам. Для камеры нужно запускать FSDS с обновлённым
`simulation/fsds-settings.json`; после изменения settings FSDS надо перезапустить.

Если FSDS кратко зависнет или сменит карту, официальный bridge может завершиться
по `rpc::timeout`. Launch теперь автоматически перезапускает bridge и camera.
Кнопка `AUTOPILOT` ждёт свежие odometry/cones, сбрасывает safety latch через
disable и только затем включает controller. Сам FSDS при этом должен оставаться
запущенным с загруженной трассой.

Самый быстрый запуск на Windows/Linux, Python 3.8+, без ROS/GPU:

```sh
python tools/simulate.py --gui
python -m unittest discover -s tests -v
```

Без `--gui` выполняется быстрый deterministic прогон. Математика, `Bicycle`,
`circle_track` и `world_to_cones` сохранены из `ros` в `shared/car_control_core`.
FSDS остаётся внешним upstream checkout с зафиксированными commit/submodules.

Полный FSDS-профиль: Windows FSDS v2.2.0 + ROS2 Humble/Ubuntu 22.04 в WSL2,
штатный `fsds_ros2_bridge`, ground-truth cones/odom, `host:=localhost` или remote host.
Native Windows lightweight не требует WSL. Native Windows-сборка FSDS ROS2 bridge
не заявляется поддержанной: объяснение и отдельные инструкции в документации.

Jetson/ZED/TensorRT/Arduino код сохранён, simulation его не импортирует.
Аппаратный `server.py` использует общий core через `Code/Autopilot.py`.
[Существующая инструкция ROS1/Jetson](ros/README.md) относится к ROS1-профилю;
для новой FSDS-интеграции используйте ROS2-инструкцию выше.

Успешные tests не доказывают безопасность реальной машины или прохождение FSDS трассы.
Unreal GUI, производительность GPU и работа Jetson требуют ручной проверки.
При полном разрыве RPC последний газ может остаться на удалённом FSDS:
нужен оператор с доступом к остановке симулятора.
