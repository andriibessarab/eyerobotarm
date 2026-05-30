from pydobot import Dobot
from pydobot.enums import PTPMode

HOME_POS = (200.0, 100.0, 50.0, 0.0)  # x, y, z, r_head — matches provided_code/dobotArm.py


def connect(port: str) -> Dobot:
    return Dobot(port=port, verbose=False)


def initialize_robot(dobot: Dobot):
    dobot.speed(velocity=50, acceleration=50)


def move_to_xyz(dobot: Dobot, x: float, y: float, z: float, r_head: float = 0.0):
    dobot._set_ptp_cmd(x, y, z, r_head, mode=PTPMode.MOVJ_XYZ, wait=True)


def move_joint_angles(dobot: Dobot, j1: float, j2: float, j3: float, j4: float = 0.0):
    # In MOVJ_ANGLE mode the four params map to j1..j4
    dobot._set_ptp_cmd(j1, j2, j3, j4, mode=PTPMode.MOVJ_ANGLE, wait=True)


def move_to_home(dobot: Dobot):
    dobot._set_ptp_cmd(*HOME_POS, mode=PTPMode.MOVJ_XYZ, wait=True)


def rotate_end_effector(dobot: Dobot, angle: float):
    if not (-90.0 <= angle <= 90.0):
        raise ValueError(f'angle {angle} outside ±90° range')
    x, y, z, *_ = dobot.pose()
    dobot._set_ptp_cmd(x, y, z, angle, mode=PTPMode.MOVL_XYZ, wait=True)


def open_gripper(dobot: Dobot):
    dobot.grip(False)


def close_gripper(dobot: Dobot):
    dobot.grip(True)


def get_pose(dobot: Dobot) -> tuple:
    return dobot.pose()  # (x, y, z, r, j1, j2, j3, j4)
