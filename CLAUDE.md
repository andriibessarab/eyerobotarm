# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

S26 Toyota Innovation Challenge — a collaborative robotics system where a Dobot Magician arm works alongside human workers to pick and place parts (velcro tags, 3D-printed brake calipers). The system uses an Orbbec overhead camera for workspace detection and a Pi-mounted gaze camera for worker intent detection via AprilTag gaze locking.

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

### Pi gaze camera (deploy to Pi Zero 2W)

```bash
# On the Pi — streams MJPEG over HTTP on port 8080
python3 pi_scripts/stream_camera.py
# Access from main computer: http://192.168.8.2:8080/stream.mjpg
```

### Standalone provided_code (calibration / legacy)

```bash
cd provided_code
python calibrateCamera.py             # one-time intrinsic calibration
python getTransformationMatrix_linux.py  # Linux homography generation
python pickCVBlock.py                 # standalone pick-place loop
```

## Architecture

### System flow

```
┌────────────────────────────────────────────────────────────────────┐
│                          HARDWARE                                   │
│  Pi Camera (glasses) ──► gaze_camera_node (/gaze_camera/image_raw) │
│  Orbbec (overhead)   ──► workspace_camera_node                     │
│                             (/workspace_camera/image_raw)           │
└──────────────────────────────┬─────────────────────────────────────┘
                               │
              ┌────────────────┼────────────────┐
              ▼                ▼                 ▼
   apriltag_gaze_node  apriltag_workspace_node  arm_detection_node
   (tag16h5, 3s lock)  (tag→robot XY via H)    (wrist→robot XY via H)
              │                │                 │
   /gaze/gazed_tag_id  /workspace/tag_detections /workspace/arm_position
   (Int32, fires once)  (TagDetectionArray)       (Point, robot mm)
              │                │                 │
              └────────────────▼─────────────────┘
                        task_coordinator_node
                               │
                        VALIDATE both:
                        ① tag visible in overhead AND in reach?
                        ② hand visible in overhead AND in reach?
                               │ fail → IDLE + warn
                               ▼ pass
                          EXECUTING:
                          1. open gripper
                          2. move above pick       ← normal speed (50%)
                          3. descend to object     ← normal speed
                          4. close gripper
                          5. lift to Z_SAFE
                          6. move above hand       ← normal speed
                          7. reduce to 20% speed
                          8. descend to hand       ← slow (near human)
                          9. open gripper
                         10. restore speed → home → IDLE
                               │
                               ▼
                         dobot_arm_node
                         (pydobot, /dev/ttyUSB0)
                         services: move_to_xyz, gripper_control,
                                   home, set_speed, move_joints,
                                   rotate_end_effector
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
- `TagDetection.msg` — `tag_id`, `pixel_x/y`, `robot_x/y`
- `TagDetectionArray.msg` — array of `TagDetection`
- `RobotPose.msg` — `x/y/z/r`, `j1/j2/j3/j4`

**Services**
- `MoveToXYZ.srv` — `x, y, z, r_head → success, message`
- `MoveJoints.srv` — `j1, j2, j3, j4 → success, message`
- `GripperControl.srv` — `open (bool) → success`
- `SetSpeed.srv` — `velocity, acceleration (1–100%) → success`
- `PickAndPlace.srv` — defined but not used; orchestration is in task_coordinator

### Hardware layer

`dobot_arm/dobot_hardware.py` wraps `pydobot` (Linux-compatible). Serial port defaults to `/dev/ttyUSB0`; override via the `serial_port` ROS parameter. `HOME_POS = (200.0, 100.0, 50.0, 0.0)` mm.

### Coordinate system & calibration files

- `provided_code/HomographyMatrix.npy` — maps camera pixel coords to robot XY (mm). Regenerate with `getTransformationMatrix_linux.py` when camera/arm is repositioned.
- `provided_code/camera_params.npz` — intrinsic camera matrix + distortion coeffs. Regenerate with `calibrateCamera.py` only when switching cameras.
- All nodes load these at startup via the `PROVIDED_CODE_PATH` env var (set automatically by `system.launch.py`). A warning is logged if files are missing.

### task_coordinator_node parameters

| Parameter | Default | Description |
|---|---|---|
| `min_reach_mm` | `135.0` | Inner Dobot workspace bound (mm from base) |
| `max_reach_mm` | `320.0` | Outer Dobot workspace bound |
| `z_safe` | `40.0` | Clearance height (mm) for horizontal moves |
| `z_pick` | `-25.0` | Descent height (mm) to grip the object |
| `z_drop` | `0.0` | Z height (mm) for releasing into hand |
| `normal_velocity` | `50` | Arm speed % during pick and transit |
| `approach_velocity` | `20` | Arm speed % during final descent to hand |
| `exec_timeout` | `15.0` | Seconds before stuck EXECUTING resets to IDLE |

## Node status

| Node | Status |
|---|---|
| `dobot_arm_node.py` | Working — pydobot, set_speed service added |
| `task_coordinator_node.py` | Working — validation, slow approach, watchdog |
| `apriltag_gaze_node.py` | Working — cv2.aruco DICT_APRILTAG_16h5, 3s lock |
| `arm_detection_node.py` | In progress (another developer) |
| `apriltag_workspace_node.py` | In progress (another developer) |
