# Simulation: Windows, ROS2 и FSDS

## Архитектура и границы проверки

`simulation` создана от `ros` commit `2e2e053c4c00561e78ef67a5b24c46204c5f4dae`.
Ветка `ros` не изменяется, merge обратно не выполняется. Arduino bridge вне задачи.
Актуальный источник аппаратных настроек — ветка `Main` этого форка, commit
`bf04463934ccc88df5d29e100f76d0bbd4124a0f`; тесты проверяют Jetson-профиль
симуляции против `Jetson Xavier/config.jsonc` из этой истории.

```text
lightweight: circle_track → world_to_cones ──────────────┐
FSDS: official bridge → Track + Odometry → sensor adapter ├→ shared Controller
Jetson: existing perception → HardwareAutopilot ────────┘        │
                         ┌────────────────────────────────────┤
                         ↓                    ↓               ↓
                  existing Bicycle    fs_msgs command     existing hardware
                                      adapter             CarController
                                           ↓
                               /fsds/control_command
                                           ↓
                             official fsds_ros2_bridge
                                           ↓ RPC (TCP)
                                   FSDS vehicle/physics
```

`shared/car_control_core/core.py` — прежний core из `ros`, без изменения математики:
boundary interpolation, EMA, heading PID, stop-on-orange. Здесь же прежние
`Bicycle`, `circle_track`, `world_to_cones`. ROS1 импортирует его через совместимый
`car_control_ros.core`, ROS2 напрямую; аппаратный `Jetson Xavier/server.py` — через
`Code/Autopilot.py`. Параметры масштаба автомобиля различаются, алгоритм один.
`Session` добавляет watchdog и enable/reconnect lifecycle вокруг core. `sensor.py`
моделирует частоту, задержку, настраиваемые camera offsets, число детекций,
детерминированный шум и dropout. `fsds.py`
содержит только преобразования. ROS/CUDA/TensorRT/PyCUDA/ZED/serial/OpenCV импортов
в shared core нет. Hardware/perception и ROS1 custom messages сохранены.

Проверки в GitHub Actions (`Shared simulation`, `ROS simulation`): pure tests на
Windows/Linux, deterministic два круга, ROS1 Noetic catkin build/install/rostest,
сборка настоящего pinned ROS2 bridge на Humble, ROS2 DDS tests с настоящими
`fs_msgs`, но искусственными Track/Odometry/Clock; отдельный ROS2 lightweight test
без `fs_msgs`. Актуальный PASS/FAIL смотрите для своего commit.

DDS fixture сам по себе НЕ является Unreal end-to-end тестом. Дополнительно на
этом Windows 11 ноутбуке проверены FSDS v2.2.0 TrainingMap на встроенной AMD,
ROS2 bridge в WSL2, получение 196 конусов/odometry, enable/disable, движение и
финальная команда полного тормоза. Remote GPU host, реальный Jetson, полный круг
и camera inference/FPS пока не проверялись.

## Версии и upstream dependency

Источник истины — [`simulation/fsds.lock.json`](../simulation/fsds.lock.json).

| Компонент | Версия |
| --- | --- |
| FSDS | v2.2.0, `ba7b1bdac76be3895b864ca917dfc844b344c762` |
| `fs_msgs` ROS2 | `4146e5b4889fb92332c9ce5ee42a8081649cbe72` |
| rpclib | `a663a1598a4b419123b2e13c0ae6a39c91dcf5b8` |
| ROS2 bridge/controller | Humble / Ubuntu 22.04 x86-64 / Python 3.10 |
| Simulator GUI | официальный Windows x64 binary v2.2.0, Windows 11 |
| ROS1 compatibility | Noetic / Ubuntu 20.04; отдельный shell от Humble |
| Pure lightweight | Python 3.8+; CI 3.10, локально также 3.12 |

В [release notes v2.2.0](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/tag/v2.2.0)
указаны исправления Humble/Ubuntu 22.04. [Humble поддерживается до мая 2027](https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html).
Это проверяемая совместимость сборки, не обещание GUI E2E на любом ноутбуке.

FSDS НЕ скопирован внутрь проекта. `tools/prepare_fsds.py` создаёт отдельный sparse
checkout AirSim/ROS2, проверяет commit/submodules и применяет небольшой
[`fsds-2.2.0-humble.patch`](../simulation/upstream/fsds-2.2.0-humble.patch):

- системный Eigen3 и явные ament/rosgraph зависимости;
- transient-local Track: upstream публикует трассу один раз, поздний subscriber
  иначе её не получает;
- исправление origin/cm→m для конусов на мировой оси;
- watchdog 0.5 s и finite/ranges/brake проверки в штатном command callback.

Это тот же `fsds_ros2_bridge`, не новый bridge/протокол/физика. Windows binary
не модифицируется; сохраняются upstream лицензии. Helper не скачивает Unreal
assets, повторно запускается идемпотентно, чужой checkout не сбрасывает.

## Native Windows — без Linux

### Lightweight (поддержанный нативный путь)

Нужны Git и Python 3.10+; для GUI включите Tcl/Tk в установщике Python.
Не устанавливайте hardware `requirements.txt` для simulation.

```powershell
git clone --branch simulation --single-branch https://github.com/salimadze2005-beep/car_control_project_new_Danis.git
cd car_control_project_new_Danis
python tools/simulate.py
python tools/simulate.py --gui
python -m unittest discover -s tests -v
python -m unittest discover -s ros/car_control_ros/test -p "test_*.py" -v
```

GUI — 2D-трасса с синими/жёлтыми конусами, не Unreal. Без GUI 240 симулированных
секунд вычисляются быстро, dt=0.02. Контрольный результат: ~2.402 оборота,
максимальное отклонение ~0.0562 m. Это результат Bicycle, не FSDS.
PowerShell helpers (при ограниченной execution policy вызывайте Python напрямую):

```powershell
.\tools\Simulation.ps1 -Mode lightweight -Gui
.\tools\Simulation.ps1 -Mode test
.\tools\Simulation.ps1 -Mode check -FsdsHost localhost
```

### Native ROS2/FSDS bridge (не заявлен стабильным)

FSDS GUI запускается native Windows. Однако [ROS2 Humble Windows-инструкция](https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html)
нацелена на Windows 10; CMake официального FSDS ROS2 bridge использует Linux
флаги (`-pthread` и др.), Unix library paths и pkg-config/libcurl. Windows build
AirSim не доказывает Windows build ROS2 bridge. Поэтому полноценный FSDS build
здесь идёт через WSL2 fallback, а не через непроверенную «универсальную» команду.

При уже установленном пользователем native ROS2 наши Python-пакеты можно собрать
`colcon build --merge-install --base-paths shared ros2`, подключить
`install\local_setup.ps1` (либо `.bat` в соответствующем shell) и запустить
`ros2 launch car_control_sim lightweight.launch.py`. Этот native ROS2 путь
локально не проверен. `Simulation.ps1 -Mode fsds` предназначен только для
самостоятельно подготовленной native сборки upstream bridge.

## WSL2 fallback: установка и build

WSL нужен для официального C++ bridge, не для Unreal: GPU/рендеринг остаются
в Windows. Второй компьютер и GPU внутри WSL не нужны. Нет camera/TensorRT
inference в ROS2-контуре на этом этапе: Jetson baseline моделирует измерения из
ground truth. При remote режиме controller также не требует CUDA.

PowerShell администратора, если WSL ещё не установлен:

```powershell
wsl --install -d Ubuntu-22.04
```

При необходимости перезагрузитесь. Установите ROS2 Humble по
[официальной Ubuntu deb-инструкции](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
После добавления официального ROS apt repository в Ubuntu:

```bash
sudo apt update
sudo apt install ros-humble-ros-base git build-essential cmake pkg-config \
  libeigen3-dev libyaml-cpp-dev libcurl4-openssl-dev libboost-dev libopencv-dev \
  python3-colcon-common-extensions ros-humble-ament-cmake-auto \
  ros-humble-cv-bridge ros-humble-image-transport ros-humble-tf2-ros \
  ros-humble-rosgraph-msgs ros-humble-visualization-msgs
git clone --branch simulation --single-branch https://github.com/salimadze2005-beep/car_control_project_new_Danis.git
cd car_control_project_new_Danis
bash tools/build_ros2.sh --fsds "$HOME/fsds-v2.2.0"
source simulation_ws/install/setup.bash
```

Храните Linux checkout/build в Linux filesystem, не OneDrive или `/mnt/c`:
это избегает проблем с правами/symlink/скоростью. FSDS checkout обязан быть ВНЕ
нашего repo. При нехватке RAM перед build: `export CMAKE_BUILD_PARALLEL_LEVEL=2`.
Не нужно выполнять upstream `setup.sh` или скачивать UE assets ради ROS2 build.

Только lightweight ROS2, без FSDS download:

```bash
bash tools/build_ros2.sh
source simulation_ws/install/setup.bash
ros2 launch car_control_sim lightweight.launch.py
```

Topics: `/car/sim/odom`, `/car/sim/markers`, `/car/command_debug`, `/car/status`.
Опционально установите `ros-humble-rviz2`: Fixed Frame `map`, MarkerArray
`/car/sim/markers`. `auto_start:=false` отключает автоматическое движение.
ROS2 timer использует wall time; deterministic проверки — через pure runner.

## FSDS GUI на Windows

1. Скачайте [Windows binary v2.2.0](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/download/v2.2.0/fsds-v2.2.0-windows.zip)
   и распакуйте отдельно от Git checkout. Unreal Editor не требуется.
2. Запустите `FSDS.exe`, загрузите штатную трассу, проверьте ручное движение.
   Перед ROS2 прекратите ручное управление и другие источники команд.
3. Начальный профиль без lidar/camera запросов —
   [`simulation/fsds-settings.json`](../simulation/fsds-settings.json): штатный
   Technion car, имя `FSCar`, включён GSS (нужен для `/clock`). Этот профиль
   проверен локально на TrainingMap; при проблеме начните с upstream `settings.json`.
4. Из каталога binary передайте абсолютный путь к JSON штатным `-settings`:

```powershell
$fsdsSettings = Read-Host 'Абсолютный путь к simulation\fsds-settings.json'
.\FSDS.exe -settings $fsdsSettings
```

Другие [upstream пути поиска settings](https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/blob/v2.2.0/docs/getting-started.md)
тоже поддержаны. Bridge получает settings непосредственно из FSDS через RPC;
копия JSON внутри WSL не нужна. Не выключайте GSS.
На встроенной AMD начните с малого окна/низкой графики. `AdapterRAM=536870912`
сам по себе не определяет всю доступную память iGPU и не позволяет обещать FPS.
Если Unreal медленный, используйте lightweight или опциональный GPU host.

## Local mode — один ноутбук, default localhost

FSDS+GPU находятся в Windows, ROS2 bridge+controller в WSL на ТОМ ЖЕ ноутбуке.
Для Windows↔WSL localhost используйте mirrored networking на Windows 11 22H2+
с актуальным WSL. В `%UserProfile%\.wslconfig` добавьте к существующим настройкам:

```ini
[wsl2]
networkingMode=mirrored
```

Перезапустите WSL: `wsl --shutdown` завершает ВСЕ WSL-сессии, сначала сохраните
работу. [Microsoft: mirrored, localhost и NAT](https://learn.microsoft.com/en-us/windows/wsl/networking).
После запуска FSDS и загрузки трассы, из sourced ROS2 shell:

```bash
python3 tools/check_fsds.py --host localhost
ros2 launch car_control_sim fsds.launch.py
```

Default `host:=localhost` передаётся в штатный параметр bridge `host_ip`.
Во втором WSL shell из репозитория:

```bash
source simulation_ws/install/setup.bash
ros2 topic echo /car/status
```

Когда доступны track, odom, clock и GO, в отдельном sourced shell:

```bash
ros2 service call /car/enable std_srvs/srv/SetBool "{data: true}"
# Остановить:
ros2 service call /car/enable std_srvs/srv/SetBool "{data: false}"
```

Controller стартует DISABLED и публикует brake. GO от bridge — дополнительный
heartbeat, не замена пользовательского enable и не аппаратный safety interlock.

Проверенный для WSL2 профиль FSDS использует `bridge_telemetry_period:=0.05`:
20 Hz для telemetry и controller command. Это необходимо, потому что upstream
bridge использует однопоточный executor; его штатные 250 Hz odom polling вместе
с 50 Hz command могут starving callback управления через Windows↔WSL RPC.
Обычный `fsds.launch.py` использует governor 2 m/s и brake 0.25. Jetson-профиль
использует target 1 m/s, полный brake, soft zone 1 m/s и ранний brake margin
0.8 m/s. Core выдаёт тот же нормализованный throttle=1.0, что аппаратный
`HardwareAutopilot`; `fsds_throttle_scale:=0.20` преобразует его в input PhysX.
Это калибровка исполнительного адаптера, а не другая математика controller.

```bash
ros2 launch car_control_sim fsds.launch.py host:="$FSDS_HOST" \
  fsds_max_speed_mps:=1.5 fsds_throttle_scale:=0.20
```

После изменения исходного JSON запустите `tools/build_ros2.sh` снова или передайте
абсолютный source path через `controller_config:=...`: launch обычно читает копию
config из `simulation_ws/install`.

Если mirrored недоступен, WSL NAT — всё ещё LOCAL режим на одном компьютере.
В `settings.json` на Windows укажите `LocalHostIp` ровно как Windows gateway из
WSL (на проверенном ноутбуке `172.21.112.1`), затем перезапустите FSDS.

```bash
FSDS_HOST=$(ip route show default | awk '{print $3; exit}')
python3 tools/check_fsds.py --host "$FSDS_HOST"
ros2 launch car_control_sim fsds.launch.py host:="$FSDS_HOST"
```

Нужно узкое firewall-разрешение для WSL IP, не отключение Firewall.

## Remote FSDS — опциональный GPU host

На другом Windows PC/облачном GPU host установите тот же FSDS v2.2.0 binary,
загрузите трассу. FSDS использует локальную GPU ТОГО компьютера, GPU не
объединяются и не передаются по сети. В его JSON `LocalHostIp: "0.0.0.0"`,
`ApiServerPort: 41451`, затем перезапуск FSDS. Предпочтительно приватная LAN/VPN.
На ноутбуке bridge/controller остаются вместе; меняется один параметр:

```bash
read -r -p 'FSDS IP/hostname: ' FSDS_HOST
python3 tools/check_fsds.py --host "$FSDS_HOST"
ros2 launch car_control_sim fsds.launch.py host:="$FSDS_HOST"
```

Enable/disable те же. Host — только IP/hostname, без `http://` и `:41451`.
Custom API port штатным bridge этого профиля не поддержан, оставьте 41451.
DDS остаётся на ноутбуке, наружу идёт FSDS RPC. Нет `ROS_MASTER_URI` (это ROS1),
Arduino server или собственного networking. RDP/Parsec/Moonlight — только GUI;
control/sensors передаются напрямую, не через видеострим.

### Windows Firewall / облако

На FSDS host от администратора разрешите только TCP 41451 для фактической
программы и адреса controller/VPN. Пример запрашивает реальные значения:

```powershell
$fsdsExecutable = Read-Host 'Полный путь к реальному FSDS/FSOnline exe, слушающему порт'
$controllerAddress = Read-Host 'IP ноутбука/VPN (для NAT — IP WSL)'
New-NetFirewallRule -DisplayName 'FSDS RPC from controller' -Direction Inbound `
  -Action Allow -Protocol TCP -LocalPort 41451 -Program $fsdsExecutable `
  -RemoteAddress $controllerAddress -Profile Private
```

PID процесса: `Get-NetTCPConnection -LocalPort 41451`, затем `Get-Process -Id <PID>`;
launcher может отличаться от FSOnline. В облаке ограничьте ingress/security group
адресом VPN/controller. Не открывайте неаутентифицированный RPC всему Internet.
При mirrored может также действовать Hyper-V firewall (Microsoft link выше).
Проверка из Windows: `Test-NetConnection -ComputerName localhost -Port 41451`.

## Контракты адаптера и параметры

| FSDS v2.2.0 ROS2 | Shared core / действие |
| --- | --- |
| BLUE=0 / YELLOW=1 | blue / yellow: синяя слева, жёлтая справа |
| ORANGE_BIG=2 / ORANGE_SMALL=3 | orange; stop policy настраивается |
| UNKNOWN=4, неизвестные enum, NaN/Inf | отбросить конус |
| Track location | уже метры, ENU, origin в точке старта; НЕ повторять cm→m |
| Odometry | `fsds/map` → `fsds/FSCar`, position m, quaternion → yaw rad |
| vehicle forward | `cos(yaw)*dx + sin(yaw)*dy` |
| core right | `sin(yaw)*dx - cos(yaw)*dy`, минус ROS body-left |
| throttle | [0,1], нормализованный газ, НЕ m/s |
| steering | [-1,1], плюс вправо, НЕ rad; знак совпадает с core |
| brake | [0,1]; при brake>0 throttle принудительно 0 |
| non-finite command | throttle=0, steering=0, brake=1 |

Используется прежний `world_to_cones`, FOV 90°, дальность из config. Неверные frames
или quaternion приводят к остановке. Проекция плоская, не проверка склонов/roll/pitch.
Track — статическая карта; freshness проверяется по odom и clock, не по времени
публикации Track. После смены карты перезапускайте launch для новой карты.

`ros2/car_control_sim/config/fsds.json`: core throttle 1.0, lookahead 3 m,
width 3 m, depth 15 m. На этом FSDS команды 0.10, 0.15 и 0.18 не обеспечили
устойчивое трогание TechnionCar, а 0.20 запустила его, поэтому adapter по умолчанию
умножает core throttle на 0.20. Это калибровка конкретной PhysX-модели, не
значение для Jetson.
`fsds_max_speed_mps` (default 2.0) и `fsds_speed_brake` (default 0.25) — governor
только FSDS adapter: core math и lightweight/hardware не меняются. `stop_on_orange=false`
в FSDS намеренно: оранжевые отмечают старт/финиш. Для нужного сценария включите
его в своём JSON; lightweight/hardware сохраняют прежний stop policy.

```bash
ros2 launch car_control_sim fsds.launch.py host:=localhost controller_config:=/absolute/path/to/config.json fsds_max_speed_mps:=1.5
```

## Jetson baseline и stress simulation

Обычный `fsds.launch.py` остаётся ideal ground-truth режимом. Профиль
`fsds_jetson.launch.py` использует controller-параметры из `Main`:

- track width 1.5 m, lookahead 0.5 m, range 0.1..4.0 m;
- PID 1.0/0.1/0.25, EMA 0.85, integral limit 1.5;
- 15 observations/s, не более 6 ближайших конусов каждого цвета;
- camera correction X=0.0 m, Z=0.10 m и один camera period задержки (0.067 s);
- target FSDS speed 1.0 m/s, full brake, soft zone 1.0 m/s и brake margin 0.8 m/s.

В `Main/config.jsonc` записан `camera_offset_x=-0.06`, однако текущий
`Main/server.py` загружает, но не применяет его; реально применяется только
`z - camera_offset_z`. Поэтому baseline повторяет работающий код с X=0.0. После
исправления Jetson pipeline включите ту же коррекцию в симуляции параметром
`sensor_camera_offset_x_m:=-0.06`.

`fsds_max_speed_mps` — порог governor, а не гарантированный физический максимум.
У полноразмерного TechnionCar есть порог статического трения: 0.18 газа ещё не
трогает машину, а 0.20 вместе с WSL/RPC/telemetry delay даёт короткий выбег примерно
до 2 m/s даже при target 1 m/s. Поэтому smoke ceiling 2.5 m/s проверяет отсутствие
неконтролируемого разгона, но не объявляет модель динамически эквивалентной
маленькой машине. Для переноса speed tuning нужны логи реального разгона/торможения.

Оранжевая остановка в baseline выключена намеренно: штатные FSDS-трассы используют
оранжевые конусы также на старте. Это единственное controller-policy отличие от
реального `Main`; включать stop следует на отдельном FSDS сценарии с корректной
расстановкой финишных конусов.

```bash
ros2 launch car_control_sim fsds_jetson.launch.py host:="$FSDS_HOST"
```

Шум и пропуски по умолчанию равны нулю: без записанных Jetson-логов их величину
нельзя считать калиброванной. Для воспроизводимого stress test:

```bash
ros2 launch car_control_sim fsds_jetson.launch.py host:="$FSDS_HOST" \
  sensor_dropout_probability:=0.05 \
  sensor_lateral_std_m:=0.02 \
  sensor_depth_relative_std:=0.08 \
  sensor_seed:=2005
```

Значения stress test проверяют устойчивость, но не являются измерениями ZED.
Следующий шаг к sensor-in-the-loop — камера FSDS → ONNX → те же cone tuples;
baseline нужен как контролируемая ступень между ideal ground truth и perception.

## ONNX → TensorRT engine на Jetson

Переданный `best.onnx` не копируется в Git. Его проверенные метаданные находятся в
`simulation/model-manifest.json`: SHA-256
`063a90fa56e01fa405a810d5550b76ddc04fc51ede9885de57b6611b7bb49fde`, статический
input `images` 1x3x640x640, output `output0`, NMS не встроен, классы
0=yellow, 1=orange, 2=blue. Это соответствует текущему `Cone_detector.py` и
`Main/config.jsonc`. Встроенные metadata модели указывают Ultralytics 8.4.16 и
AGPL-3.0; перед распространением бинарника отдельно проверьте лицензионные условия.

TensorRT engine нельзя корректно собрать на AMD-ноутбуке и переносить на Jetson:
engine зависит от GPU architecture, TensorRT и JetPack. Скопируйте ONNX на целевой
Jetson и собирайте там тем же окружением, в котором запускается `server.py`:

```bash
cd ~/car_control_project_new_Danis
python3 tools/build_tensorrt_engine.py ~/best.onnx ~/cone_detector_best_fp16.engine \
  --expected-sha256 063a90fa56e01fa405a810d5550b76ddc04fc51ede9885de57b6611b7bb49fde
```

Helper ищет `trtexec` в PATH и `/usr/src/tensorrt/bin/trtexec`, выбирает актуальный
workspace flag для TensorRT 8/10, собирает FP16 и оставляет включённым штатный
inference benchmark. Существующий engine не перезаписывается без `--force`.
Успешный `trtexec` доказывает загрузку/выполнение engine, но не качество детекции.
После сборки измените `vision.yolo_model_path` в `Jetson Xavier/config.jsonc` и
проверьте модель на записанном видео до подключения исполнительных механизмов.

## Safety и границы гарантий

- Старт disabled; нет track/валидных cones/GO/odom — throttle=0, brake=1.
- Stale odom >0.4 s по receipt/source, замерший clock, rewind, потеря GO >4 s после
  движения дают fault; восстановление связи не включает движение, нужен enable.
- Timer работает по steady wall clock, замерший `/clock` watchdog не выключает.
- Invalid command → brake; конечные значения насыщаются до допустимых диапазонов.
  Brake имеет приоритет, stop-on-orange latch в core сохранён.
- Смерть controller при живом bridge: патч штатного bridge после ~0.5 s без команд
  пытается отправить brake своим RPC client. Это не hard real-time гарантия:
  блокирующий RPC/timeout влияет на задержку.
- Полная потеря RPC/bridge/controller host НЕ гарантирует остановку удалённого
  FSDS: upstream PhysXCarApi хранит последнюю команду. Наш wrapper прекращает
  выдавать throttle при потере данных, но brake нельзя доставить через обрыв.
  После reconnect сначала проверьте/остановите FSDS, перезапустите bridge, затем enable.

Network-failure эксперименты требуют оператора с pause/reset/закрытием FSDS на GPU
host. Гарантированный remote stop потребовал бы watchdog внутри simulator binary
(НЕ реализован и не заявляется). Не подключайте hardware actuators к FSDS topics.

## Tests и ручная приёмка

Без ROS/GPU/hardware:

```bash
python3 -m unittest discover -s tests -v
python3 -m unittest discover -s ros/car_control_ros/test -p 'test_*.py' -v
python3 tools/simulate.py --seconds 240
```

Pure tests включают sync Jetson controller profile, sensor rate/offset/latency,
детерминированный noise/dropout и построение команды `trtexec`.

Humble после build/source:

```bash
python3 tests/ros2_lightweight.py
python3 tests/ros2_graph.py  # требует сгенерированный upstream fs_msgs
```

При запущенных Windows FSDS и `fsds_jetson.launch.py` безопасный motion smoke:

```bash
python3 tests/fsds_smoke.py
```

Он требует disabled+brake до старта, на восемь секунд включает controller,
проверяет движение, throttle и runaway ceiling 2.5 m/s, а в `finally` всегда
вызывает disable и проверяет финальный полный brake. Для диагностики добавьте
`--trace`; уменьшенный ceiling можно задавать вручную, но текущая PhysX-модель
не гарантирует 1.5 m/s из-за описанного выше выбега.

ROS1 compatibility в отдельной Noetic/Ubuntu 20.04 среде:

```bash
bash ros/build.sh
source ros_ws/devel/setup.bash
cd ros_ws
catkin_make run_tests
catkin_test_results
catkin_make install
```

Ручная FSDS приёмка (что ещё остаётся проверить):

1. GUI v2.2.0, settings/FSCar/GSS, TCP check успешно, launch без RPC errors.
2. Сначала проехать ideal `fsds.launch.py`, затем на скорости не выше 1 m/s
   повторить `fsds_jetson.launch.py`; stress включать только после baseline.
3. Проверить Track (также late subscriber), odom, clock, GO:
   `ros2 topic echo /fsds/testing_only/track --once --qos-durability transient_local`.
4. До enable brake=1; после enable короткий заезд, визуально проверить цвета,
   повороты вправо/влево, метры/геометрию и соответствие сцены траектории.
5. Disable тормозит. Смерть только controller при живом bridge вызывает brake.
   Pause/stale odom не удерживает throttle; reconnect сам движение не включает.
6. Отдельно local и remote, если нужен. Полный обрыв сети — только с оператором,
   учитывая ограничение последней команды. Записать FPS, карту, параметры, круги.
7. FSDS параметры нельзя переносить на реальную машину без аппаратных испытаний.

## Troubleshooting

- TCP check failed: GUI в меню/не слушает, bind/settings, NAT localhost, VPN/firewall.
  Helper проверяет только TCP, не версию протокола и не readiness FSDS.
- `fs_msgs`/bridge not found: build нужен с `--fsds`, source нужного install;
  не смешивайте ROS1 environment с ROS2.
- Нет Track: competition_mode=false, namespace `/fsds`, нужен QoS patch. После
  смены карты перезапустите bridge. Не исправляйте это изменением core.
- `stale_or_no_data`: GSS/clock, odom source/frames. При
  `connection_fault_reenable_required` восстановите источники, затем enable.
- Warning про nodes с одинаковым именем или нестабильные команды: одновременно
  запущены несколько `fsds.launch.py`/`fsds_jetson.launch.py`. Сначала disable,
  корректно остановите лишний launch и оставьте один controller/bridge.
- `no_usable_cones`: вне трассы/FOV/дальности или неверный origin; сначала reset
  FSDS и новая Track, потом настройка geometry.
- Build OOM: CMAKE_BUILD_PARALLEL_LEVEL=2; UE assets для bridge не нужны.
- Patch не применяется: проверьте pin и локальные правки upstream; helper их не
  сбрасывает. Выберите новый внешний каталог.
- Нет tkinter: Python с Tcl/Tk либо запуск без GUI.
- CUDA/Jetson import errors: запущен hardware server вместо simulation helper;
  hardware requirements для этих simulation-режимов не нужны.
