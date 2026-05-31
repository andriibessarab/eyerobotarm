# task_coordinator

ROS2 package containing the high-level orchestration node that converts a gaze-confirmed worker intent into a physical pick-and-place operation on the Dobot arm.

---

## Node: `task_coordinator_node`

### What it does

Implements a two-state machine (`IDLE` → `EXECUTING` → `IDLE`) that listens for a gaze-locked AprilTag ID and, when one arrives, moves the arm to pick that tag's part and drop it at the detected worker hand position.

**Trigger:** `apriltag_gaze_node` publishes a non-negative `tag_id` on `/gaze/gazed_tag_id` when the worker has held their gaze on a tag for the required stability time. This node treats any non-negative value as a confirmed intent — no additional stability check here.

**Pick position:** looked up by `tag_id` in the latest `/workspace/tag_detections` message (published by `apriltag_workspace_node`). If the tag is not visible in the overhead camera, the pick is aborted with a warning.

**Drop position:** taken from the latest `/workspace/arm_position` message (the detected wrist position from `arm_detection_node`). Falls back to `drop_fallback_x/y` parameters if no arm position has been received.

### Pick-and-place sequence

Chained async service calls to `dobot_arm_node`:

```
open gripper
  → move to (pick_x, pick_y, Z_SAFE)
  → move to (pick_x, pick_y, Z_PICK)
  → close gripper
  → move to (pick_x, pick_y, Z_SAFE)
  → move to (drop_x, drop_y, Z_SAFE)
  → open gripper
  → home
  → IDLE
```

`Z_SAFE = 40.0 mm`, `Z_PICK = -25.0 mm` (hardcoded constants matching `provided_code/pickCVBlock.py`).

While in `EXECUTING` state, any new gaze-lock events are ignored.

---

### Topics

| Direction | Topic | Type | Description |
|---|---|---|---|
| Subscribes | `/gaze/gazed_tag_id` | `std_msgs/Int32` | Gaze lock trigger from `apriltag_gaze_node` |
| Subscribes | `/workspace/tag_detections` | `pick_interfaces/msg/TagDetectionArray` | AprilTag positions in robot frame |
| Subscribes | `/workspace/arm_position` | `geometry_msgs/Point` | Detected wrist position in pixel space |
| Publishes | `~/state` | `std_msgs/String` | Current state (`IDLE` or `EXECUTING`), published at 1 Hz |

### Service clients

| Service | Type | When called |
|---|---|---|
| `/dobot_arm_node/move_to_xyz` | `pick_interfaces/srv/MoveToXYZ` | Each move step |
| `/dobot_arm_node/gripper_control` | `pick_interfaces/srv/GripperControl` | Open/close steps |
| `/dobot_arm_node/home` | `std_srvs/srv/Trigger` | Final step after drop |

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `drop_fallback_x` | float | `200.0` | Robot X (mm) to drop at if no wrist detected |
| `drop_fallback_y` | float | `0.0` | Robot Y (mm) to drop at if no wrist detected |

---

## Running

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --packages-select pick_interfaces task_coordinator
source install/setup.bash

ros2 run task_coordinator task_coordinator_node
```

Requires `dobot_arm_node`, `apriltag_workspace_node`, and `apriltag_gaze_node` to be running. Monitor state:

```bash
ros2 topic echo /task_coordinator_node/state
```
