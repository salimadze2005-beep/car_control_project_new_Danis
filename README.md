# car_control_project_new_Danis — simulation

Один controller core для Jetson/ROS1, лёгкого симулятора и FSDS/ROS2.
Ветка создана от `ros`; обратно ничего не сливается.

**[Установка и запуск simulation → docs/SIMULATION_SETUP.md](docs/SIMULATION_SETUP.md)**

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
