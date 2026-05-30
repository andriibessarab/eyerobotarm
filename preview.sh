#!/bin/bash
# Launch camera stream + detection + live preview window (no arm required).
# Usage: bash ~/Desktop/S26-Toyota-Innovation-Challenge/preview.sh

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/jazzy/setup.bash
source "$REPO/ros2_ws/install/setup.bash"

cleanup() {
    echo ""
    echo "Shutting down..."
    kill $CAM_PID $DET_PID $PREV_PID 2>/dev/null
    wait $CAM_PID $DET_PID $PREV_PID 2>/dev/null
    exit 0
}
trap cleanup SIGINT SIGTERM

# Release the camera device if something is already holding it
fuser -k /dev/video0 2>/dev/null
sleep 0.5

ros2 run camera_pipeline workspace_camera_node &
CAM_PID=$!
sleep 2

ros2 run camera_pipeline object_detection_node &
DET_PID=$!
sleep 1

ros2 run camera_pipeline preview_node &
PREV_PID=$!

echo "Running — close the preview window or press Ctrl+C to stop."
wait $PREV_PID
cleanup
