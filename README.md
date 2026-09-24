# car_control_project_new_Danis — ветка `simulation`

Полный гайд по переносу проекта на **другой ПК с нуля**: как скачать именно ветку `simulation`, установить окружение, собрать ROS2/FSDS bridge, запустить симулятор, проверить связь и включить ручной/автономный режим.

> **Основная проверенная схема:** Windows 11 x64 + WSL2 Ubuntu 22.04 + ROS2 Humble + FSDS v2.2.0.  
> FSDS/Unreal работает в Windows, ROS2 bridge и controller — в WSL2.  
> Для быстрой проверки логики без ROS/FSDS есть lightweight-режим на обычном Python.

Подробности архитектуры и тестирования:

- [docs/SIMULATION_SETUP.md](docs/SIMULATION_SETUP.md) — расширенная документация FSDS/ROS2;
- [docs/SIMULATION_TESTING.md](docs/SIMULATION_TESTING.md) — трассы, запись и проверки;
- [simulation/fsds.lock.json](simulation/fsds.lock.json) — точные pinned-версии upstream FSDS и submodules.

---

## 1. Что именно устанавливается

Проект состоит из нескольких частей:

```text
Windows
├─ FSDS v2.2.0 (Unreal simulator)
├─ Python 3.x
└─ FSDS CONTROL panel / launcher

WSL2 Ubuntu 22.04
├─ ROS2 Humble
├─ pinned FSDS ROS2 bridge
├─ car_control_sim
├─ shared/car_control_core
└─ controller / recorder / helper tools
```

### Полный FSDS-режим требует

- Windows 11 x64;
- WSL2;
- Ubuntu 22.04;
- ROS2 Humble;
- Git;
- Windows Python 3.10+;
- готовый FSDS v2.2.0 Windows binary;
- доступ к GitHub во время первой установки.

### Для lightweight-режима достаточно

- Git;
- Python 3.8+.

Lightweight не требует ROS, WSL, CUDA, TensorRT, Jetson, Arduino или GPU.

---

## 2. Важные ограничения перед установкой

1. Для полной FSDS-схемы **репозиторий лучше хранить в Linux filesystem WSL**, например:

   ```text
   /home/<user>/car_control_project_new_Danis
   ```

   Не рекомендуется собирать ROS2 workspace из `/mnt/c`, OneDrive или сетевой папки.

2. Upstream FSDS ROS2 source checkout должен быть **отдельно от этого репозитория**.  
   Рекомендуемый путь:

   ```text
   /home/<user>/fsds-v2.2.0
   ```

3. Не ставьте hardware-зависимости Jetson/ZED/TensorRT только ради симуляции.

4. Для полной схемы используйте именно:
   - Ubuntu **22.04**;
   - ROS2 **Humble**;
   - FSDS **v2.2.0**.

5. Ветка проекта:

   ```text
   simulation
   ```

---

# Часть A. Быстрая проверка проекта без ROS и FSDS

Этот вариант нужен, чтобы убедиться, что код скачался и базовая логика работает.

## 3. Установить Git и Python на Windows

Установите:

- Git for Windows;
- Python 3.10+ с официального python.org.

При установке Python желательно включить:

- `Add Python to PATH`;
- `tcl/tk and IDLE` — требуется для GUI.

Проверка в PowerShell:

```powershell
git --version
python --version
```

## 4. Скачать именно ветку `simulation`

```powershell
cd $HOME
git clone --branch simulation --single-branch https://github.com/salimadze2005-beep/car_control_project_new_Danis.git
cd car_control_project_new_Danis
git branch --show-current
```

Ожидаемый результат последней команды:

```text
simulation
```

## 5. Запустить lightweight simulation

Без GUI:

```powershell
python tools/simulate.py
```

С GUI:

```powershell
python tools/simulate.py --gui
```

Тесты:

```powershell
python -m unittest discover -s tests -v
python -m unittest discover -s ros/car_control_ros/test -p "test_*.py" -v
```

Или через PowerShell helper:

```powershell
.\tools\Simulation.ps1 -Mode lightweight -Gui
.\tools\Simulation.ps1 -Mode test
```

Если это работает — базовый Python-контур проекта установлен правильно.

---

# Часть B. Полная установка FSDS + ROS2 на новом Windows ПК

## 6. Установить WSL2 + Ubuntu 22.04

Откройте **PowerShell от имени администратора**:

```powershell
wsl --install -d Ubuntu-22.04
```

После установки Windows может попросить перезагрузку.

После перезагрузки запустите Ubuntu 22.04 и создайте Linux user/password.

Проверка из PowerShell:

```powershell
wsl -l -v
```

Ожидаемо Ubuntu должна быть версии WSL 2.

Если указано `1`:

```powershell
wsl --set-version Ubuntu-22.04 2
```

---

## 7. Установить ROS2 Humble внутри Ubuntu 22.04

Все команды ниже выполняются **в терминале Ubuntu/WSL**, не в Windows PowerShell.

### 7.1. Locale и базовые пакеты

```bash
sudo apt update
sudo apt install -y locales software-properties-common curl

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo add-apt-repository universe
sudo apt update
```

### 7.2. Подключить официальный ROS2 apt repository

Актуальный рекомендуемый ROS способ — пакет `ros2-apt-source`:

```bash
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')

curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"

sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
```

Официальная инструкция ROS2 Humble:

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

### 7.3. Установить ROS2 и зависимости проекта

```bash
sudo apt install -y \
  ros-humble-ros-base \
  git build-essential cmake pkg-config \
  libeigen3-dev libyaml-cpp-dev libcurl4-openssl-dev libboost-dev libopencv-dev \
  python3-numpy python3-opencv python3-colcon-common-extensions \
  ros-humble-ament-cmake-auto \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-tf2-ros \
  ros-humble-rosgraph-msgs \
  ros-humble-visualization-msgs
```

Проверка:

```bash
source /opt/ros/humble/setup.bash
ros2 --help
python3 --version
git --version
```

Можно автоматически source ROS2 в каждом новом shell:

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
```

---

## 8. Скачать проект в WSL

Рекомендуется отдельный Linux checkout, даже если ранее проект уже скачивался в Windows.

```bash
cd ~
git clone --branch simulation --single-branch \
  https://github.com/salimadze2005-beep/car_control_project_new_Danis.git

cd ~/car_control_project_new_Danis
git branch --show-current
git status
```

Ожидаемая ветка:

```text
simulation
```

---

## 9. Собрать ROS2 workspace и pinned FSDS bridge

Проект фиксирует точные upstream версии в `simulation/fsds.lock.json`.

Выполните:

```bash
cd ~/car_control_project_new_Danis
source /opt/ros/humble/setup.bash

bash tools/build_ros2.sh --fsds "$HOME/fsds-v2.2.0"
```

Что делает команда:

1. создаёт внешний sparse checkout FSDS v2.2.0 в `~/fsds-v2.2.0`;
2. проверяет pinned commit и submodules;
3. применяет project patches для Humble/bridge/camera/watchdog;
4. собирает:
   - `shared/car_control_core`;
   - `ros2/car_control_sim`;
   - upstream `fs_msgs`;
   - upstream `fsds_ros2_bridge`;
5. создаёт workspace `simulation_ws`.

После успешной сборки:

```bash
source ~/car_control_project_new_Danis/simulation_ws/install/setup.bash
```

Для удобства можно добавить source workspace в `~/.bashrc`:

```bash
echo 'source ~/car_control_project_new_Danis/simulation_ws/install/setup.bash' >> ~/.bashrc
```

> Добавляйте эту строку только **после первой успешной сборки**.

### Если во время build не хватает RAM

```bash
export CMAKE_BUILD_PARALLEL_LEVEL=2
bash tools/build_ros2.sh --fsds "$HOME/fsds-v2.2.0"
```

---

## 10. Проверить сборку до запуска Unreal/FSDS

В WSL:

```bash
cd ~/car_control_project_new_Danis
source /opt/ros/humble/setup.bash
source simulation_ws/install/setup.bash

python3 -m unittest discover -s tests -v
python3 tests/ros2_lightweight.py
python3 tests/ros2_graph.py
```

Дополнительно можно запустить ROS2 lightweight:

```bash
ros2 launch car_control_sim lightweight.launch.py
```

Остановить: `Ctrl+C`.

---

## 11. Скачать FSDS v2.2.0 на Windows

Используется готовый Windows binary FSDS v2.2.0:

https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/tag/v2.2.0

Прямая ссылка на используемый архив:

https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/download/v2.2.0/fsds-v2.2.0-windows.zip

Распакуйте, например, в:

```text
C:\FSDS
```

Проверьте, что существует:

```text
C:\FSDS\FSDS.exe
```

Полный Unreal Editor для обычного запуска симуляции не нужен.

---

# Часть C. Первый полный запуск на новом ПК

Самый простой первый запуск — через generated test track. Launcher сам создаст отдельный settings snapshot и подставит адрес Windows/WSL.

## 12. Узнать WSL username и путь к репозиторию из Windows

Откройте обычный PowerShell:

```powershell
$Distro = "Ubuntu-22.04"
$WslUser = (wsl -d $Distro -- bash -lc 'printf %s "$USER"').Trim()
$RepoWin = "\\wsl.localhost\$Distro\home\$WslUser\car_control_project_new_Danis"

$WslUser
$RepoWin
```

Пример результата:

```text
user
\\wsl.localhost\Ubuntu-22.04\home\user\car_control_project_new_Danis
```

Это устраняет старую жёсткую привязку к `/home/danis`.

---

## 13. Запустить тестовую трассу FSDS

Перед запуском закройте все старые `FSDS.exe` / `Blocks.exe`.

В Windows PowerShell:

```powershell
python "$RepoWin\tools\launch_fsds_test.py" \
  --track test_ground \
  --fsds-directory C:\FSDS
```

Доступные трассы:

```text
test_ground
figure_eight
slalom
turns
```

Например:

```powershell
python "$RepoWin\tools\launch_fsds_test.py" --track figure_eight --fsds-directory C:\FSDS
```

Launcher напечатает примерно:

```text
FSDS PID=...
track=test_ground
host=172.x.x.x
Run files: C:\FSDS\test_runs\...
WSL: ros2 launch car_control_sim fsds_drive.launch.py host:=172.x.x.x
```

**Скопируйте значение `host`.**

Если FSDS уже запущен, launcher специально откажется стартовать вторую копию.

---

## 14. Запустить ROS2 bridge + controller

Откройте новый WSL terminal:

```bash
cd ~/car_control_project_new_Danis
source /opt/ros/humble/setup.bash
source simulation_ws/install/setup.bash
```

Подставьте адрес, который вывел Windows launcher:

```bash
ros2 launch car_control_sim fsds_drive.launch.py host:=172.x.x.x
```

Если WSL mirrored networking настроен и `localhost` действительно работает:

```bash
ros2 launch car_control_sim fsds_drive.launch.py host:=localhost
```

Профиль `fsds_drive` специально стартует в **MANUAL**, чтобы машина не поехала неожиданно.

---

## 15. Проверить TCP связь с FSDS

До launch или при проблемах:

```bash
cd ~/car_control_project_new_Danis
python3 tools/check_fsds.py --host 172.x.x.x
```

Ожидаемо:

```text
TCP reachable: ...:41451
```

Это проверяет только TCP-порт. Полную работу подтверждают ROS topics/odom/track/clock.

---

## 16. Запустить Windows-панель управления

Панель должна запускаться **Windows Python**, а не WSL Python.

В том же Windows PowerShell, где уже заданы `$Distro`, `$WslUser`, `$RepoWin`:

```powershell
pythonw "$RepoWin\tools\fsds_control_panel.py" \
  --distro $Distro \
  --repository "/home/$WslUser/car_control_project_new_Danis"
```

Если `pythonw` не находится:

```powershell
python "$RepoWin\tools\fsds_control_panel.py" \
  --distro $Distro \
  --repository "/home/$WslUser/car_control_project_new_Danis"
```

Панель имеет режимы:

- **MANUAL** — машина управляется стрелками;
- **AUTOPILOT** — shared controller управляет по конусам;
- **STOP** — controller выключен, bridge отправляет тормоз.

Если нажать стрелку во время AUTOPILOT, панель возвращает управление в MANUAL.

---

## 17. Управление без GUI-панели

Во втором WSL terminal:

```bash
cd ~/car_control_project_new_Danis
source /opt/ros/humble/setup.bash
source simulation_ws/install/setup.bash

python3 tools/fsds_mode.py manual
python3 tools/fsds_mode.py auto
python3 tools/fsds_mode.py stop
```

Статус:

```bash
ros2 topic echo /car/status
```

---

## 18. Настройка скорости и steering во время запуска

Пример:

```bash
python3 tools/fsds_speed.py 2.5 \
  --throttle-scale 0.20 \
  --steering-gain 0.80 \
  --steering-response 0.50 \
  --steering-limit 0.70
```

Проверенные значения профиля `fsds_drive` по умолчанию:

```text
target speed       = 2.5 m/s
throttle scale     = 0.20
steering gain      = 0.80
steering response  = 0.50
steering limit     = 0.70
```

Можно задать параметры сразу при launch:

```bash
ros2 launch car_control_sim fsds_drive.launch.py \
  host:="$FSDS_HOST" \
  fsds_max_speed_mps:=2.5 \
  fsds_throttle_scale:=0.20
```

Не увеличивайте параметры резко: физика FSDS инерционная, а большие gain/throttle могут ухудшить устойчивость.

---

## 19. Запись камеры и телеметрии

Через панель:

- `START RECORD`;
- `STOP RECORD`.

Или из WSL:

```bash
python3 tools/fsds_record.py start
python3 tools/fsds_record.py status
python3 tools/fsds_record.py stop
```

Результат создаётся в:

```text
recordings/YYYYMMDD_HHMMSS_ffffff/
```

Внутри:

- `camera.mp4`;
- `frames/`;
- `frames.csv`;
- `telemetry.csv`;
- `events.jsonl`;
- `summary.json`;
- `report.md`;
- `recorder.log`.

---

# Часть D. Обычный запуск без generated test track

Если хотите открыть штатную карту FSDS вручную:

1. запустите `C:\FSDS\FSDS.exe`;
2. загрузите карту;
3. убедитесь, что FSDS использует settings с GSS и front camera;
4. в WSL определите host;
5. запустите `fsds_drive.launch.py`.

Для WSL NAT:

```bash
FSDS_HOST=$(ip route show default | awk '{print $3; exit}')
echo "$FSDS_HOST"

python3 tools/check_fsds.py --host "$FSDS_HOST"

ros2 launch car_control_sim fsds_drive.launch.py host:="$FSDS_HOST"
```

Для mirrored networking:

```bash
python3 tools/check_fsds.py --host localhost
ros2 launch car_control_sim fsds_drive.launch.py host:=localhost
```

Актуальный project settings находится здесь:

```text
simulation/fsds-settings.json
```

---

# Часть E. WSL mirrored networking — опционально

На Windows 11 можно включить mirrored networking.

Файл:

```text
%UserProfile%\.wslconfig
```

Содержимое:

```ini
[wsl2]
networkingMode=mirrored
```

После изменения:

```powershell
wsl --shutdown
```

Затем снова запустите Ubuntu.

> `wsl --shutdown` закрывает все WSL-сессии.

Mirrored mode удобен тем, что Windows ↔ WSL часто могут работать через `localhost`. Если он не нужен или не работает, используйте NAT host, который автоматически определяет project launcher.

---

# Часть F. Обновление проекта на уже настроенном ПК

В WSL:

```bash
cd ~/car_control_project_new_Danis

git checkout simulation
git pull --ff-only origin simulation

source /opt/ros/humble/setup.bash
bash tools/build_ros2.sh --fsds "$HOME/fsds-v2.2.0"

source simulation_ws/install/setup.bash
```

После изменений ROS2 Python/config/launch файлов желательно:

1. пересобрать workspace;
2. остановить старый launch;
3. заново source `simulation_ws/install/setup.bash`;
4. запустить launch снова;
5. перезапустить control panel.

---

# Часть G. Проверка установки — короткий checklist

На полностью новом ПК установка считается базово готовой, если выполняются все пункты:

```text
[ ] Windows видит: C:\FSDS\FSDS.exe
[ ] wsl -l -v показывает Ubuntu-22.04 / VERSION 2
[ ] source /opt/ros/humble/setup.bash работает
[ ] ros2 --help работает
[ ] git branch --show-current -> simulation
[ ] bash tools/build_ros2.sh --fsds ~/fsds-v2.2.0 завершился без ошибки
[ ] source simulation_ws/install/setup.bash работает
[ ] python3 tests/ros2_lightweight.py проходит
[ ] FSDS test launcher запускает карту
[ ] tools/check_fsds.py видит TCP 41451
[ ] ros2 launch car_control_sim fsds_drive.launch.py подключается к FSDS
[ ] control panel открывается
[ ] MANUAL / AUTOPILOT / STOP переключаются
[ ] STOP оставляет машину заторможенной
```

---

# Часть H. Частые проблемы

## `FSDS.exe` не найден

Проверьте:

```text
C:\FSDS\FSDS.exe
```

Или укажите другой путь:

```powershell
python "$RepoWin\tools\launch_fsds_test.py" \
  --track test_ground \
  --fsds-directory "D:\Programs\FSDS"
```

---

## `ros2: command not found`

В текущем WSL terminal:

```bash
source /opt/ros/humble/setup.bash
```

После build также:

```bash
source ~/car_control_project_new_Danis/simulation_ws/install/setup.bash
```

---

## `fs_msgs` или `fsds_ros2_bridge` не найден

Пересоберите **с `--fsds`**:

```bash
cd ~/car_control_project_new_Danis
source /opt/ros/humble/setup.bash
bash tools/build_ros2.sh --fsds "$HOME/fsds-v2.2.0"
source simulation_ws/install/setup.bash
```

---

## Build падает по памяти

```bash
export CMAKE_BUILD_PARALLEL_LEVEL=2
bash tools/build_ros2.sh --fsds "$HOME/fsds-v2.2.0"
```

---

## Control panel ищет `/home/danis/...`

Передайте правильный путь явно:

```powershell
pythonw "$RepoWin\tools\fsds_control_panel.py" \
  --distro Ubuntu-22.04 \
  --repository "/home/$WslUser/car_control_project_new_Danis"
```

---

## `TCP connection failed`

1. Убедитесь, что FSDS реально открыт и карта загружена.
2. Проверьте host:
   ```bash
   ip route show default
   ```
3. Проверьте:
   ```bash
   python3 tools/check_fsds.py --host "$FSDS_HOST"
   ```
4. Не отключайте Windows Firewall полностью. При необходимости создайте узкое правило на TCP 41451.
5. VPN может менять маршрутизацию Windows/WSL.

---

## `rpc::timeout`

Проверьте, что одновременно нет двух старых копий FSDS.

В Windows PowerShell:

```powershell
Get-Process FSDS,Blocks -ErrorAction SilentlyContinue
```

Должна работать только нужная копия симулятора.

После смены карты рекомендуется перезапустить ROS2 launch.

---

## `no_usable_cones`

Это означает, что controller не видит подходящий коридор конусов.

Попробуйте:

1. переключиться в MANUAL;
2. поставить машину стрелками вдоль трассы;
3. убедиться, что синие/жёлтые конусы впереди;
4. затем снова включить AUTOPILOT.

---

## GUI lightweight не запускается

Проверьте Tk:

```powershell
python -m tkinter
```

Если окно не появляется — переустановите Windows Python с Tcl/Tk.

---

## WSL distro называется не `Ubuntu-22.04`

Проверьте:

```powershell
wsl -l -v
```

И передайте реальное имя:

```powershell
$Distro = "ВАШЕ_ИМЯ_DISTRO"
```

Control panel:

```powershell
pythonw "$RepoWin\tools\fsds_control_panel.py" --distro $Distro --repository "/home/$WslUser/car_control_project_new_Danis"
```

---

# Часть I. Что НЕ нужно устанавливать для FSDS simulation

Для обычного `fsds_drive` не нужны:

- Jetson Xavier;
- ZED SDK;
- CUDA;
- TensorRT;
- PyCUDA;
- Arduino;
- SolidWorks;
- полный Unreal Editor;
- аппаратный `Jetson Xavier/server.py`;
- hardware requirements реальной машины.

FSDS controller использует ground-truth Track/Odometry через официальный ROS2 bridge.

---

# Часть J. Структура важных файлов

```text
README.md
docs/
├─ SIMULATION_SETUP.md
└─ SIMULATION_TESTING.md

simulation/
├─ fsds.lock.json
├─ fsds-settings.json
├─ model-manifest.json
├─ tracks/
└─ upstream/

shared/
└─ car_control_core/

ros2/
└─ car_control_sim/
   ├─ config/
   ├─ launch/
   └─ car_control_sim/

tools/
├─ build_ros2.sh
├─ prepare_fsds.py
├─ launch_fsds_test.py
├─ fsds_control_panel.py
├─ fsds_mode.py
├─ fsds_speed.py
├─ fsds_record.py
├─ check_fsds.py
└─ simulate.py

tests/
```

---

# Часть K. Безопасность и границы

Этот FSDS-контур — симуляционный.

- Не подключайте hardware actuators реальной машины к FSDS topics.
- Перед AUTOPILOT сначала проверьте MANUAL и STOP.
- При полном разрыве RPC/сети simulator может некоторое время сохранять последнюю принятую команду — у оператора должен оставаться доступ к pause/reset/закрытию FSDS.
- Для remote FSDS не открывайте TCP 41451 всему Интернету; используйте private LAN/VPN и firewall rule только для нужного адреса.
- Успешные unit/ROS tests не являются доказательством безопасной работы реального автомобиля.

---

## Самый короткий путь после уже выполненной установки

### Windows PowerShell

```powershell
$Distro = "Ubuntu-22.04"
$WslUser = (wsl -d $Distro -- bash -lc 'printf %s "$USER"').Trim()
$RepoWin = "\\wsl.localhost\$Distro\home\$WslUser\car_control_project_new_Danis"

python "$RepoWin\tools\launch_fsds_test.py" --track test_ground --fsds-directory C:\FSDS
```

Скопируйте напечатанный `host`.

### WSL

```bash
cd ~/car_control_project_new_Danis
source /opt/ros/humble/setup.bash
source simulation_ws/install/setup.bash

ros2 launch car_control_sim fsds_drive.launch.py host:=АДРЕС_ИЗ_WINDOWS_LAUNCHER
```

### Windows PowerShell — control panel

```powershell
pythonw "$RepoWin\tools\fsds_control_panel.py" \
  --distro $Distro \
  --repository "/home/$WslUser/car_control_project_new_Danis"
```

После этого:

1. `MANUAL`;
2. поставьте машину вдоль трассы;
3. `AUTOPILOT`;
4. для остановки — `STOP`.

---

**Ветка:** `simulation`  
**FSDS:** v2.2.0  
**ROS2:** Humble  
**Ubuntu:** 22.04  
**Основной full-simulation target:** Windows 11 + WSL2
