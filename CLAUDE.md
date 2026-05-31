# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

S26 Toyota Innovation Challenge — a collaborative robotics system where a Dobot Magician arm works alongside human workers to pick and place parts (velcro tags, 3D-printed brake calipers). The system uses an Orbbec overhead camera for workspace detection and a gaze camera for worker intent detection via AprilTag gaze locking.

Two parallel codebases:
- **`provided_code/`** — standalone Python scripts (no ROS2) that are the ground-truth for calibration and CV logic.
- **`ros2_ws/`** — a ROS2 workspace that wraps the provided code into composable nodes.

## Build & Run

```bash
# Source ROS2 first
source /opt/ros/jazzy/setup.bash

# Build from workspace root
cd ros2_ws
colcon build
source install/setup.bash

# Launch the full system
ros2 launch launch/system.launch.py

# Run a single node
ros2 run camera_pipeline arm_detection_node
ros2 run camera_pipeline apriltag_gaze_node
ros2 run camera_pipeline apriltag_workspace_node
ros2 run camera_pipeline preview_node   # live OpenCV debug window
ros2 run dobot_arm dobot_arm_node
ros2 run task_coordinator task_coordinator_node
```

### Standalone provided_code (calibration / legacy)

```bash
cd provided_code
python calibrateCamera.py             # one-time intrinsic calibration
python getTransformationMatrix_linux.py  # Linux homography generation
python getTransformationMatrix.py     # Windows homography generation
python pickCVBlock.py                 # standalone pick-place loop
```

## Architecture

### ROS2 node graph

```
workspace_camera_node ──► /workspace_camera/image_raw ──┬──► apriltag_workspace_node ──► /workspace/tag_detections
                                                         ├──► arm_detection_node ──────► /workspace/arm_position
                                                         └──► preview_node (debug window)

gaze_camera_node ──────► /gaze_camera/image_raw ──────────► apriltag_gaze_node ──────► /gaze/gazed_tag_id (Int32)

/gaze/gazed_tag_id ──────────────────────────────────────┐
/workspace/tag_detections ───────────────────────────────┼──► task_coordinator_node ──► dobot_arm_node
/workspace/arm_position ─────────────────────────────────┘        ~/move_to_xyz
                                                                   ~/move_joints
                                                                   ~/gripper_control
                                                                   ~/home
                                                                   ~/rotate_end_effector
```

### Packages

| Package | Nodes |
|---|---|
| `pick_interfaces` | msgs/srvs only (C++ CMake) |
| `camera_pipeline` | `workspace_camera_node`, `gaze_camera_node`, `apriltag_workspace_node`, `apriltag_gaze_node`, `arm_detection_node`, `preview_node` |
| `dobot_arm` | `dobot_arm_node` + `dobot_hardware.py` abstraction |
| `task_coordinator` | `task_coordinator_node` |

### Custom interfaces (`pick_interfaces`)

**Messages**
- `ObjectDetection.msg` — `header`, `label`, `pixel_x/y`, `robot_x/y`, `confidence`
- `RobotPose.msg` — `x/y/z/r`, `j1/j2/j3/j4`
- `TagDetection.msg` — `tag_id`, `pixel_x/y`, `robot_x/y`
- `TagDetectionArray.msg` — array of `TagDetection`

**Services**
- `MoveToXYZ.srv` — `x, y, z, r_head → success, message`
- `MoveJoints.srv` — `j1, j2, j3, j4 → success, message`
- `GripperControl.srv` — `open (bool) → success`
- `PickAndPlace.srv` — `pick_x/y, drop_x/y → success, message`

### Hardware layer

`dobot_arm/dobot_hardware.py` wraps `pydobot` (Linux-compatible, no DLL needed). Serial port defaults to `/dev/ttyUSB0`; override via the `serial_port` ROS parameter. `HOME_POS = (200.0, 100.0, 50.0, 0.0)`.

The `provided_code/lib/DobotDll.dll` is Windows-only and is NOT used by the ROS2 stack.

### Coordinate system & calibration files

- `provided_code/HomographyMatrix.npy` — maps camera pixel coords to robot XY (mm). Regenerate with `getTransformationMatrix_linux.py` when camera/arm is repositioned.
- `provided_code/H_matrix.json` — same homography in JSON (alternate format).
- `provided_code/camera_params.npz` — intrinsic camera matrix + distortion coeffs. Regenerate with `calibrateCamera.py` only when switching cameras.
- All nodes that need calibration load these files at startup via the `PROVIDED_CODE_PATH` env var (defaults to `../provided_code` relative to the node file). A warning is logged if files are missing; detection is silently skipped.

### Key constants (`task_coordinator_node.py`)

```python
Z_SAFE = 40.0   # mm — clearance height for horizontal moves
Z_PICK = -25.0  # mm — height to grip object
```

### Pick-and-place flow

`task_coordinator_node` state machine: `IDLE → EXECUTING → IDLE`.

Trigger: `apriltag_gaze_node` publishes a non-negative `tag_id` on `/gaze/gazed_tag_id` when a tag has been stably gazed at. The coordinator looks up the matching tag's robot XY in `/workspace/tag_detections` (from `apriltag_workspace_node`), uses the latest `/workspace/arm_position` as the drop target (falls back to `drop_fallback_x/y` params), then chains async service calls: open gripper → move safe → move pick → close gripper → lift safe → move drop → open gripper → home.

## What is stubbed / needs implementation

| Node | Status |
|---|---|
| `arm_detection_node.py` | Working — wrist tracking via MediaPipe Hands |
| `dobot_arm_node.py` | Working — uses `pydobot` via `dobot_hardware.py` |
| `task_coordinator_node.py` | Working — full async pick-and-place chain implemented |
| `apriltag_gaze_node.py` | **TODO** — `_cb_frame` stub; needs `cv2.aruco.detectMarkers` + stability counter |
| `apriltag_workspace_node.py` | **TODO** — `_cb_frame` stub; needs detection + publish `TagDetectionArray` |
