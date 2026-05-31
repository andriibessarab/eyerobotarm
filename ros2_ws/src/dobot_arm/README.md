# dobot_arm

ROS2 node that wraps the Dobot Magician arm over a USB serial connection using [pydobot](https://github.com/luismesas/pydobot).

---

## Setup

### Dependencies

```bash
pip install pydobot pyserial
```

### Build

```bash
source /opt/ros/jazzy/setup.bash   # adjust distro if needed
cd ros2_ws
colcon build --packages-select pick_interfaces dobot_arm
source install/setup.bash
```

### USB port permissions (Linux)

The arm shows up as `/dev/ttyUSB0`. Your user must be in the `dialout` group:

```bash
sudo usermod -aG dialout $USER   # log out and back in after
```

---

## Running the node

```bash
ros2 run dobot_arm dobot_arm_node --ros-args -p serial_port:=/dev/ttyUSB0
```

On startup the node opens the serial port, sets speed/acceleration to 50 % of max, and prints `dobot_arm_node ready`. The arm does **not** auto-home on startup — call the `/home` service explicitly if needed.

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `serial_port` | string | `/dev/ttyUSB0` | Serial port the arm is connected to |
| `rotate_angle` | float | `0.0` | Angle (degrees) used by the `rotate_end_effector` service |

---

## Services

All services are advertised under the node name: `/dobot_arm_node/<service>`.

### `/dobot_arm_node/home`

Move the arm to the home position (200, 100, 50 mm).

- **Type:** `std_srvs/srv/Trigger`
- **Request:** *(none)*
- **Response:** `success` (bool), `message` (string)

```bash
ros2 service call /dobot_arm_node/home std_srvs/srv/Trigger {}
```

---

### `/dobot_arm_node/move_to_xyz`

Move the arm to a Cartesian position using joint-interpolated PTP motion.

- **Type:** `pick_interfaces/srv/MoveToXYZ`
- **Request:** `x` (float64, mm), `y` (float64, mm), `z` (float64, mm), `r_head` (float64, degrees)
- **Response:** `success` (bool), `message` (string)

```bash
ros2 service call /dobot_arm_node/move_to_xyz \
  pick_interfaces/srv/MoveToXYZ \
  "{x: 200.0, y: 50.0, z: 50.0, r_head: 0.0}"
```

---

### `/dobot_arm_node/move_joints`

Move the arm to specific joint angles.

- **Type:** `pick_interfaces/srv/MoveJoints`
- **Request:** `j1` (float64, deg), `j2` (float64, deg), `j3` (float64, deg), `j4` (float64, deg)
- **Response:** `success` (bool), `message` (string)

```bash
ros2 service call /dobot_arm_node/move_joints \
  pick_interfaces/srv/MoveJoints \
  "{j1: 0.0, j2: 45.0, j3: 45.0, j4: 0.0}"
```

---

### `/dobot_arm_node/gripper_control`

Open or close the gripper.

- **Type:** `pick_interfaces/srv/GripperControl`
- **Request:** `open` (bool) — `true` to release, `false` to grip
- **Response:** `success` (bool)

```bash
# Open
ros2 service call /dobot_arm_node/gripper_control \
  pick_interfaces/srv/GripperControl "{open: true}"

# Close
ros2 service call /dobot_arm_node/gripper_control \
  pick_interfaces/srv/GripperControl "{open: false}"
```

---

### `/dobot_arm_node/set_speed`

Set arm velocity and acceleration. Called by `task_coordinator_node` before the slow final approach to the human hand.

- **Type:** `pick_interfaces/srv/SetSpeed`
- **Request:** `velocity` (int32, 1–100 %), `acceleration` (int32, 1–100 %)
- **Response:** `success` (bool)

```bash
ros2 service call /dobot_arm_node/set_speed \
  pick_interfaces/srv/SetSpeed "{velocity: 20, acceleration: 20}"
```

---

### `/dobot_arm_node/rotate_end_effector`

Rotate the end effector to the angle set in the `rotate_angle` parameter. Valid range: −90° to +90°. The arm holds its current XY position and only changes the R axis.

- **Type:** `std_srvs/srv/Trigger`
- **Request:** *(none — angle is read from the `rotate_angle` ROS parameter)*
- **Response:** `success` (bool), `message` (string)

```bash
# Set the target angle first, then call
ros2 param set /dobot_arm_node rotate_angle 45.0
ros2 service call /dobot_arm_node/rotate_end_effector std_srvs/srv/Trigger {}
```

---

## Published topics

### `/dobot_arm_node/pose`

Live pose of the arm, published at 10 Hz.

- **Type:** `pick_interfaces/msg/RobotPose`
- **Fields:** `x`, `y`, `z`, `r` (Cartesian mm/deg), `j1`, `j2`, `j3`, `j4` (joint angles, deg)

```bash
ros2 topic echo /dobot_arm_node/pose
```

---

## Architecture

```
dobot_arm_node
  │
  ├── dobot_hardware.py   (pydobot adapter)
  │     connect()              → opens /dev/ttyUSB0 at 115200 baud
  │     initialize_robot()     → set speed/accel to 50 %
  │     move_to_xyz()          → PTPMode.MOVJ_XYZ
  │     move_joint_angles()    → PTPMode.MOVJ_ANGLE
  │     move_to_home()         → PTPMode.MOVJ_XYZ to (200, 100, 50, 0)
  │     rotate_end_effector()  → PTPMode.MOVL_XYZ (R axis only)
  │     open_gripper()         → grip(False)
  │     close_gripper()        → grip(True)
  │     get_pose()             → (x, y, z, r, j1, j2, j3, j4)
  │
  └── provided_code/dobotArm.py   (Windows/DLL reference — not used on Linux)
```

`dobot_hardware.py` is a thin adapter that mirrors the `dobotArm.py` function signatures so the node code stays clean. The underlying pydobot library communicates over the same serial protocol as the original Windows DLL.
