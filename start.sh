#!/bin/bash
# Launch the full S26 Toyota Innovation Challenge system.
# Run from any directory: bash ~/Desktop/S26-Toyota-Innovation-Challenge/start.sh

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/jazzy/setup.bash
source "$REPO/ros2_ws/install/setup.bash"

echo "Starting system..."
echo "  - dobot_arm_node    (arm on /dev/ttyUSB0)"
echo "  - workspace_camera  (Orbbec on /dev/video0)"
echo "  - object_detection  (plate + target detection)"
echo "  - task_coordinator  (state machine)"
echo ""
echo "Monitor detections in another terminal:"
echo "  ros2 topic echo /object_detection_node/detected_objects"
echo ""

ros2 launch "$REPO/ros2_ws/launch/system.launch.py"
