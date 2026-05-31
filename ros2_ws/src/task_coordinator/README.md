# task_coordinator

ROS2 package containing the high-level orchestration node that converts a gaze-confirmed worker intent into a validated, safe pick-and-place operation on the Dobot arm.

---

## Node: `task_coordinator_node`

### What it does

Two-state machine (`IDLE` → `EXECUTING` → `IDLE`) with an upfront validation gate:

1. `apriltag_gaze_node` publishes a non-negative `tag_id` after the worker has held their gaze on a tag for the required stability time.
2. **Validate** (synchronous, cached data):
   - Is the gazed tag visible in the overhead camera AND within Dobot reach?
   - Is a human hand visible in the overhead camera AND within Dobot reach?
   - If either fails → log warning and stay IDLE.
3. **Execute** pick-and-place:
   - Pick object at tag's robot XY position.
   - Transit to above the detected hand at normal speed.
   - Slow down to approach speed, descend to hand height.
   - Release and home.
4. A watchdog timer resets to IDLE if any service call hangs (arm disconnected, etc.).

### Pick-and-place sequence

```
open gripper
→ move above pick (Z_SAFE=40mm)    ← normal speed
→ descend to pick (Z_PICK=-25mm)   ← normal speed
→ close gripper
→ lift to Z_SAFE
→ move above hand (Z_SAFE)         ← normal speed
→ set speed to approach_velocity
→ descend to hand (Z_DROP=0mm)     ← slow (near human)
→ open gripper
→ restore normal_velocity
→ home → IDLE
```

---

### Topics

| Direction | Topic | Type | Description |
|---|---|---|---|
| Subscribes | `/gaze/gazed_tag_id` | `std_msgs/Int32` | Gaze lock from `apriltag_gaze_node` |
| Subscribes | `/workspace/tag_detections` | `pick_interfaces/msg/TagDetectionArray` | Tag positions in robot frame |
| Subscribes | `/workspace/arm_position` | `geometry_msgs/Point` | Hand position in robot frame (mm) |
| Publishes | `~/state` | `std_msgs/String` | `IDLE` or `EXECUTING` at 1 Hz |

### Service clients

| Service | Type | When called |
|---|---|---|
| `/dobot_arm_node/move_to_xyz` | `pick_interfaces/srv/MoveToXYZ` | Each move step |
| `/dobot_arm_node/gripper_control` | `pick_interfaces/srv/GripperControl` | Open/close steps |
| `/dobot_arm_node/set_speed` | `pick_interfaces/srv/SetSpeed` | Before slow approach, after release |
| `/dobot_arm_node/home` | `std_srvs/srv/Trigger` | Final step |

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `min_reach_mm` | float | `135.0` | Inner bound of Dobot workspace (mm from base) |
| `max_reach_mm` | float | `320.0` | Outer bound of Dobot workspace |
| `z_drop` | float | `0.0` | Z height (mm) to release into hand |
| `normal_velocity` | int | `50` | Arm speed % for pick and transit |
| `approach_velocity` | int | `20` | Arm speed % for final descent near human |
| `exec_timeout` | float | `15.0` | Seconds before EXECUTING auto-resets to IDLE |

**Hardcoded Z heights** (constants in source — tune by editing `task_coordinator_node.py`):

| Constant | Value | Description |
|---|---|---|
| `Z_SAFE` | `40.0 mm` | Clearance height for all horizontal moves |
| `Z_PICK` | `-25.0 mm` | Descent height to grip the object |

---

## Running

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --packages-select pick_interfaces task_coordinator
source install/setup.bash

ros2 run task_coordinator task_coordinator_node
```

Monitor state:

```bash
ros2 topic echo /task_coordinator_node/state
```

Test validation + watchdog (no arm required):

```bash
# Trigger with in-reach tag and hand
ros2 topic pub /workspace/tag_detections pick_interfaces/msg/TagDetectionArray \
  "{detections: [{tag_id: 1, robot_x: 200.0, robot_y: 50.0}]}" --once
ros2 topic pub /workspace/arm_position geometry_msgs/Point \
  "{x: 180.0, y: 80.0, z: 0.0}" --once
ros2 topic pub /gaze/gazed_tag_id std_msgs/Int32 "{data: 1}" --once
# Expected: EXECUTING → IDLE after ~15s timeout (no arm connected)
```
