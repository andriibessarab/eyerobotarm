# camera_pipeline

ROS2 package containing all camera-facing nodes: raw frame publishers, computer-vision detectors, and a live preview window.

---

## Nodes

### `workspace_camera_node`

Captures frames from the overhead fixed camera (the one with a computed homography matrix) and publishes them at 30 Hz.

| | |
|---|---|
| **Publishes** | `/workspace_camera/image_raw` (`sensor_msgs/Image`, bgr8) |
| **Parameter** | `camera_source` (string, default `'0'`) — device index or URL |

---

### `gaze_camera_node`

Captures frames from the glasses-mounted gaze camera and publishes them at 30 Hz.

| | |
|---|---|
| **Publishes** | `/gaze_camera/image_raw` (`sensor_msgs/Image`, bgr8) |
| **Parameter** | `camera_source` (string, default `'1'`) — device index (`'0'`, `'1'`, …) or full URL (`'http://192.168.8.2:8080/stream.mjpg'`) |

---

### `apriltag_workspace_node`

Detects all `tag16h5` AprilTags visible in the overhead workspace camera, converts each tag's pixel centre to robot XY, and publishes a `TagDetectionArray`.

| | |
|---|---|
| **Subscribes** | `/workspace_camera/image_raw` |
| **Publishes** | `/workspace/tag_detections` (`pick_interfaces/msg/TagDetectionArray`) |
| **Env var** | `PROVIDED_CODE_PATH` (same as above) |

> **Status:** detection callback (`_cb_frame`) is not yet implemented — publishes nothing until filled in.

---

### `apriltag_gaze_node`

Detects `tag16h5` AprilTags in the gaze camera feed and determines which tag the worker is looking at by applying a centre-proximity + time-based stability filter. Publishes `-1` on every frame until a tag has been stably centred for `stare_time` seconds, then fires the locked `tag_id` once and resets.

Uses `cv2.aruco` with `DICT_APRILTAG_16h5` (OpenCV's equivalent of `tag16h5`).

| | |
|---|---|
| **Subscribes** | `/gaze_camera/image_raw` |
| **Publishes** | `/gaze/gazed_tag_id` (`std_msgs/Int32`) — positive tag ID on lock, `-1` otherwise |

| Parameter | Type | Default | Description |
|---|---|---|---|
| `stare_time` | float | `3.0` | Seconds a tag must remain centred before locking |
| `center_tolerance` | int | `150` | Max pixel distance from frame centre for a tag to qualify |
| `center_offset_y` | int | `50` | Upward pixel offset applied to tag centre before distance calc (accounts for glasses mount angle) |

---

### `arm_detection_node`

Detects the human operator's wrist position in the overhead workspace camera using MediaPipe Hands, converts the wrist pixel position to robot XY via the homography matrix, and publishes a `geometry_msgs/Point`. Only publishes when a hand is confidently detected.

| | |
|---|---|
| **Subscribes** | `/workspace_camera/image_raw` |
| **Publishes** | `/workspace/arm_position` (`geometry_msgs/Point`, x/y in pixels; z unused) |
| **Env var** | `PROVIDED_CODE_PATH` (same as above) |

MediaPipe Hands is configured with `max_num_hands=1`, `min_detection_confidence=0.8`.

---

### `preview_node`

Opens a live OpenCV window showing the workspace camera feed with the detected wrist position overlaid as a red dot. Useful for debugging without a separate visualizer. Press **Q** in the window to quit.

| | |
|---|---|
| **Subscribes** | `/workspace_camera/image_raw`, `/workspace/arm_position` |
| **Renders** | OpenCV window at 30 fps, scaled to 50 % of native resolution |

---

## Running

```bash
# Build
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --packages-select pick_interfaces camera_pipeline
source install/setup.bash

# Run individual nodes
ros2 run camera_pipeline workspace_camera_node --ros-args -p camera_index:=0
ros2 run camera_pipeline gaze_camera_node       --ros-args -p camera_index:=1
ros2 run camera_pipeline apriltag_workspace_node
ros2 run camera_pipeline apriltag_gaze_node
ros2 run camera_pipeline arm_detection_node
ros2 run camera_pipeline preview_node
```

## Testing `apriltag_gaze_node` standalone

1. Print or display a `tag16h5` AprilTag (search "AprilTag 16h5 generator").
2. Run a camera node pointed at the tag:
   ```bash
   ros2 run camera_pipeline gaze_camera_node --ros-args -p camera_index:=0
   ```
3. Run the gaze node:
   ```bash
   ros2 run camera_pipeline apriltag_gaze_node
   ```
4. Watch the output:
   ```bash
   ros2 topic echo /gaze/gazed_tag_id
   ```
   Hold the tag centred in frame for ~3 s — one positive tag ID fires, then returns to `-1`.

## Dependencies

```bash
pip install opencv-contrib-python mediapipe
```

`opencv-contrib-python` is required (not `opencv-python`) for the `cv2.aruco` module.
