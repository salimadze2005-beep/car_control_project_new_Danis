#!/usr/bin/env bash
set -eo pipefail
CAR_REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
CAR_WS="${CAR_WS:-$CAR_REPO/simulation_ws}"
mkdir -p "$CAR_WS"
cd "$CAR_WS"
if [[ "${1:-}" == "--fsds" ]]; then
  FSDS_ROOT="${2:?Pass the external pinned FSDS checkout}"
  python3 "$CAR_REPO/tools/prepare_fsds.py" --destination "$FSDS_ROOT"
  colcon build --base-paths "$CAR_REPO/shared" "$CAR_REPO/ros2" \
    "$FSDS_ROOT/ros2/src/fs_msgs" "$FSDS_ROOT/ros2/src/fsds_ros2_bridge" \
    --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
else
  colcon build --base-paths "$CAR_REPO/shared" "$CAR_REPO/ros2"
fi
echo "Built. Source $CAR_WS/install/setup.bash"
