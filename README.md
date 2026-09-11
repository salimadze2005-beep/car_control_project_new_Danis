# car_control_project_new_Danis — simulation

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
- Добавлены unit, ROS1, ROS2, lightweight и FSDS motion-smoke проверки.

## Быстрый запуск FSDS

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

- `MANUAL` — стрелки управляют машиной.
- `AUTOPILOT` — shared controller ведёт машину по конусам.
- `STOP` — controller выключен, bridge отправляет тормоз.
- Стрелка во время `AUTOPILOT` автоматически возвращает `MANUAL`.

Без панели переключение выполняется из sourced WSL shell:

```bash
python3 tools/fsds_mode.py manual
python3 tools/fsds_mode.py auto
python3 tools/fsds_mode.py stop
```

## Скорость и `throttle_scale`

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

`fsds_max_speed_mps` — целевая скорость governor в м/с, не жёсткое физическое
ограничение: из-за PhysX на коротком разгоне возможен небольшой выбег выше target.
`throttle_scale` — множитель нормализованной команды газа shared controller перед
передачей в FSDS: при `0.20` команда core `1.0` становится газом FSDS `0.20`.

- Увеличение `throttle_scale` делает разгон сильнее и быстрее, но повышает выбег,
  риск колебаний скорости и схода в повороте.
- Уменьшение делает разгон мягче; слишком малое значение не преодолеет статическое
  трение виртуальной TechnionCar и машина не тронется.
- Сначала меняйте scale маленькими шагами `0.02`, затем проверяйте заезд:
  `python3 tests/fsds_smoke.py --seconds 15 --trace`.

Параметры геометрии, PID, дальности и цветов находятся в
`ros2/car_control_sim/config/fsds_drive.json`. Точный профиль текущей Jetson
машины отдельно сохранён в `fsds_jetson.launch.py`.

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
