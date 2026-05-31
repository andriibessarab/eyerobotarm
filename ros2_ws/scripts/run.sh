#!/bin/bash
set -e
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

source /opt/ros/jazzy/setup.bash
cd "$REPO/ros2_ws"

if [[ "$1" == "--build" ]]; then
    colcon build --symlink-install 2>&1 | tail -4
fi

source install/setup.bash
ros2 launch "$REPO/ros2_ws/launch/system.launch.py"
