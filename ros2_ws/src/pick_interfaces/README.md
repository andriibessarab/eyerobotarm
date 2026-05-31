# pick_interfaces

ROS2 C++ CMake package containing all custom message and service definitions used across the system.

---

## Messages

### `TagDetection.msg`

One detected AprilTag from the `apriltag_workspace_node`.

| Field | Type | Description |
|---|---|---|
| `tag_id` | `int32` | AprilTag ID |
| `pixel_x` | `float64` | Tag centre X in image pixels |
| `pixel_y` | `float64` | Tag centre Y in image pixels |
| `robot_x` | `float64` | Tag centre X in robot frame (mm) |
| `robot_y` | `float64` | Tag centre Y in robot frame (mm) |

---

### `TagDetectionArray.msg`

Array of `TagDetection` entries, published once per frame by `apriltag_workspace_node`.

| Field | Type | Description |
|---|---|---|
| `header` | `std_msgs/Header` | Timestamp of the source frame |
| `detections` | `TagDetection[]` | All tags detected in this frame |

---

### `RobotPose.msg`

Live pose of the Dobot arm, published at 10 Hz by `dobot_arm_node`.

| Field | Type | Description |
|---|---|---|
| `x` | `float64` | End-effector X (mm) |
| `y` | `float64` | End-effector Y (mm) |
| `z` | `float64` | End-effector Z (mm) |
| `r` | `float64` | End-effector rotation (degrees) |
| `j1`–`j4` | `float64` | Joint angles (degrees) |

---

## Services

### `MoveToXYZ.srv`

Move the arm to a Cartesian position using joint-interpolated PTP motion.

```
float64 x
float64 y
float64 z
float64 r_head
---
bool    success
string  message
```

### `MoveJoints.srv`

Move the arm to specific joint angles.

```
float64 j1
float64 j2
float64 j3
float64 j4
---
bool   success
string message
```

### `GripperControl.srv`

Open or close the gripper.

```
bool open    # true = release, false = grip
---
bool success
```

### `PickAndPlace.srv`

High-level pick-and-place (defined but not currently called by any node — orchestration is done directly in `task_coordinator_node`).

```
float64 pick_x
float64 pick_y
float64 drop_x
float64 drop_y
---
bool   success
string message
```

---

## Build

`pick_interfaces` must be built before any Python package that imports from it:

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --packages-select pick_interfaces
source install/setup.bash
```
