#!/usr/bin/env bash
set -eo pipefail
HEXAPOD_WS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ ! -f "${HEXAPOD_WS_DIR}/install/setup.bash" ]]; then
  echo "请先运行：cd ${HEXAPOD_WS_DIR} && colcon build --symlink-install" >&2
  exit 1
fi
source "${HEXAPOD_WS_DIR}/install/setup.bash"
export ROS_LOG_DIR="${HEXAPOD_WS_DIR}/log/ros"
export GAZEBO_MODEL_DATABASE_URI=""
exec ros2 launch hexapod_gazebo gazebo.launch.py "$@"
