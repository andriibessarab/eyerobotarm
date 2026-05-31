#!/bin/bash
# Launch camera stream + detection + live preview window (no arm required).
# Usage: bash ~/Desktop/S26-Toyota-Innovation-Challenge/preview.sh

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source /opt/ros/jazzy/setup.bash
source "$REPO/ros2_ws/install/setup.bash"

CAMERA_BIN="$REPO/ros2_ws/install/camera_pipeline/lib/camera_pipeline"
USB_CAMERA_INDEX="${USB_CAMERA_INDEX:-0}"

cleanup() {
    echo ""
    echo "Shutting down..."
    kill $CAM_PID $DET_PID $ARM_PID $PREV_PID 2>/dev/null
    wait $CAM_PID $DET_PID $ARM_PID $PREV_PID 2>/dev/null
    exit 0
}
trap cleanup SIGINT SIGTERM

# Launch the USB camera as the workspace feed.
"$CAMERA_BIN/workspace_camera_node" --ros-args -p camera_index:="$USB_CAMERA_INDEX" &
CAM_PID=$!
sleep 2

"$CAMERA_BIN/object_detection_node" &
DET_PID=$!
sleep 1

"$CAMERA_BIN/arm_detection_node" &
ARM_PID=$!
sleep 1

"$CAMERA_BIN/apriltag_workspace_node" &
TAG_PID=$!
sleep 1

"$CAMERA_BIN/preview_node" &
PREV_PID=$!

echo "Running — close the preview window or press Ctrl+C to stop."
wait $PREV_PID
cleanup