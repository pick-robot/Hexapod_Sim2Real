#!/usr/bin/env bash
# 一键加载 ROS 和工作空间；策略 Python 可通过 launch 参数独立选择。
set -eo pipefail
HEXAPOD_WS_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "${HEXAPOD_WS_DIR}/install/setup.bash"
export ROS_LOG_DIR="${HEXAPOD_WS_DIR}/log/ros"
export GAZEBO_MODEL_DATABASE_URI=""
exec ros2 launch hexapod_control estimate.launch.py "$@"
