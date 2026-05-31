import time

from pydobot import Dobot
from pydobot.enums import PTPMode

HOME_POS = (200.0, 100.0, 50.0, 0.0)  # x, y, z, r_head


def connect(port: str) -> Dobot:
    return Dobot(port=port, verbose=False)


def initialize_robot(dobot: Dobot):
    dobot.speed(velocity=50, acceleration=50)


def _flush(dobot: Dobot):
    dobot.ser.reset_input_buffer()
    dobot.ser.reset_output_buffer()
    time.sleep(0.1)


def _ptp(dobot: Dobot, x, y, z, r, mode, retries=3):
    for attempt in range(retries):
        try:
            _flush(dobot)
            dobot._set_ptp_cmd(x, y, z, r, mode=mode, wait=True)
            return
        except (AttributeError, Exception) as e:
            if attempt == retries - 1:
                raise
            time.sleep(0.5)


def move_to_xyz(dobot: Dobot, x: float, y: float, z: float, r_head: float = 0.0):
    _ptp(dobot, x, y, z, r_head, PTPMode.MOVJ_XYZ)


def move_joint_angles(dobot: Dobot, j1: float, j2: float, j3: float, j4: float = 0.0):
    _ptp(dobot, j1, j2, j3, j4, PTPMode.MOVJ_ANGLE)


def move_to_home(dobot: Dobot):
    _ptp(dobot, *HOME_POS, PTPMode.MOVJ_XYZ)


def rotate_end_effector(dobot: Dobot, angle: float):
    if not (-90.0 <= angle <= 90.0):
        raise ValueError(f'angle {angle} outside ±90° range')
    x, y, z, *_ = dobot.pose()
    _ptp(dobot, x, y, z, angle, PTPMode.MOVL_XYZ)


def open_gripper(dobot: Dobot):
    dobot.grip(False)


def close_gripper(dobot: Dobot):
    dobot.grip(True)


def set_speed(dobot: Dobot, velocity: int, acceleration: int):
    dobot.speed(velocity=velocity, acceleration=acceleration)


def get_pose(dobot: Dobot) -> tuple:
    return dobot.pose()  # (x, y, z, r, j1, j2, j3, j4)
