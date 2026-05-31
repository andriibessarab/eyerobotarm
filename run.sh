#!/bin/bash
set -e
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/jazzy/setup.bash
cd "$REPO/ros2_ws"
colcon build --symlink-install 2>&1 | tail -4
source install/setup.bash

# ── Dobot serial port ──────────────────────────────────────────────────────
DOBOT_PORT="/dev/ttyUSB2"
echo "Using Dobot on $DOBOT_PORT"

# ── Arm test ───────────────────────────────────────────────────────────────
echo ""
echo "=== ARM TEST ==="
ros2 run dobot_arm dobot_arm_node --ros-args -p serial_port:="$DOBOT_PORT" &
ARM_PID=$!
sleep 3

if ! kill -0 $ARM_PID 2>/dev/null; then
    echo "ERROR: dobot_arm_node failed to start. Check serial port $DOBOT_PORT."
    exit 1
fi

echo "Opening gripper..."
ros2 service call /dobot_arm_node/gripper_control pick_interfaces/srv/GripperControl "{open: true}" || { echo "ERROR: gripper open failed"; kill $ARM_PID; exit 1; }
sleep 1

echo "Closing gripper..."
ros2 service call /dobot_arm_node/gripper_control pick_interfaces/srv/GripperControl "{open: false}" || { echo "ERROR: gripper close failed"; kill $ARM_PID; exit 1; }
sleep 1

echo "Homing..."
ros2 service call /dobot_arm_node/home std_srvs/srv/Trigger "{}" || { echo "ERROR: home failed"; kill $ARM_PID; exit 1; }
sleep 2

kill $ARM_PID 2>/dev/null
wait $ARM_PID 2>/dev/null || true
echo "=== ARM TEST PASSED ==="
echo ""

# ── Launch full system ─────────────────────────────────────────────────────
ros2 launch "$REPO/ros2_ws/launch/system.launch.py"
