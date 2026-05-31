#!/bin/bash
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$REPO/ros2_ws/install/setup.bash"

echo "Starting dobot_arm_node..."
ros2 run dobot_arm dobot_arm_node --ros-args -p serial_port:=/dev/ttyUSB2 &
ARM_PID=$!
sleep 3

echo "Testing: home..."
ros2 service call /dobot_arm_node/home std_srvs/srv/Trigger "{}"
sleep 2

echo "Testing: open gripper..."
ros2 service call /dobot_arm_node/gripper_control pick_interfaces/srv/GripperControl "{open: true}"
sleep 2

echo "Testing: close gripper..."
ros2 service call /dobot_arm_node/gripper_control pick_interfaces/srv/GripperControl "{open: false}"
sleep 2

echo "Testing: move to (200, 0, 50)..."
ros2 service call /dobot_arm_node/move_to_xyz pick_interfaces/srv/MoveToXYZ "{x: 200.0, y: 0.0, z: 50.0, r_head: 0.0}"

kill $ARM_PID 2>/dev/null
