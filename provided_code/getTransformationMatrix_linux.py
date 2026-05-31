"""
Linux-compatible homography calibration script.
Replaces getTransformationMatrix.py (which uses the Windows DLL).

Usage:
  cd provided_code
  python3 getTransformationMatrix_linux.py

Requirements:
  pip install pydobot pyserial opencv-python numpy

What it does:
  Moves the robot to 12 known positions in a 3x4 grid.
  At each position you place a red marker where the tip was.
  The script records the pixel coordinate of each marker and
  computes the homography H that maps camera pixels → robot XY (mm).
  Saves result to provided_code/HomographyMatrix.npy.

Coordinate grid (robot frame, mm):
  X: 200, 230, 260   (depth from robot base)
  Y: -80, -40, 0, 40 (left/right, positive = left when facing robot)
  Z: -24             (pick height, just above table)
"""

import sys
import time
from pathlib import Path

import cv2
import numpy as np
from pydobot import Dobot

# -----------------------------------------------------------------------
# Config
# -----------------------------------------------------------------------

SERIAL_PORT  = '/dev/ttyUSB2'
CAMERA_INDEX = 0               # Orbbec USB camera
CAL_FILE     = Path(__file__).parent / 'camera_params.npz'
OUT_FILE     = Path(__file__).parent / 'HomographyMatrix.npy'

# 12-point calibration grid (robot XY in mm)
ROBOT_POINTS = np.array([
    [180, -60], [210, -60], [240, -60],
    [180, -20], [210, -20], [240, -20],
    [180,  20], [210,  20], [240,  20],
    [180,  60], [210,  60], [240,  60],
], dtype=np.float32)

Z_CAL   = -24   # mm — height at which robot tip touches table surface
Z_CLEAR = 80    # mm — safe height to move arm out of camera view

# -----------------------------------------------------------------------
# Camera setup
# -----------------------------------------------------------------------

cam = cv2.VideoCapture(CAMERA_INDEX)
if not cam.isOpened():
    print(f'ERROR: could not open camera {CAMERA_INDEX}')
    sys.exit(1)

data = np.load(str(CAL_FILE))
camera_matrix = data['camera_matrix']
dist_coeffs   = data['dist_coeffs']

ret, frame = cam.read()
h, w = frame.shape[:2]
new_K, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1)
map1, map2 = cv2.initUndistortRectifyMap(
    camera_matrix, dist_coeffs, None, new_K, (w, h), cv2.CV_16SC2)


def undistort(f):
    return cv2.remap(f, map1, map2, cv2.INTER_LINEAR)


# -----------------------------------------------------------------------
# Red marker detection (same HSV as pickCVBlock.py)
# -----------------------------------------------------------------------

def detect_red_center(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = (cv2.inRange(hsv, np.array([0,   120, 70]), np.array([10,  255, 255])) |
            cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255])))
    mask = cv2.medianBlur(mask, 5)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    M = cv2.moments(c)
    if M['m00'] == 0:
        return None
    return int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])


# -----------------------------------------------------------------------
# Calibration loop
# -----------------------------------------------------------------------

def _move(robot, x, y, z, retries=5):
    PTPMode = __import__('pydobot.enums', fromlist=['PTPMode']).PTPMode
    for attempt in range(retries):
        try:
            robot.ser.reset_input_buffer()
            robot.ser.reset_output_buffer()
            time.sleep(0.2)
            robot._set_ptp_cmd(x, y, z, 0.0, mode=PTPMode.MOVJ_XYZ, wait=True)
            time.sleep(0.5)
            return
        except (AttributeError, Exception) as e:
            print(f'  Move failed (attempt {attempt+1}/{retries}): {e}')
            time.sleep(2.0)
    raise RuntimeError(f'Failed to move to ({x}, {y}, {z}) after {retries} attempts')




def collect_calibration(robot: Dobot) -> np.ndarray:
    pixel_points = []

    for i, (rx, ry) in enumerate(ROBOT_POINTS):
        print(f'\n── Point {i+1}/12 ── robot ({rx:.0f}, {ry:.0f}) mm')

        # Move arm to calibration point
        _move(robot, rx, ry, Z_CAL)

        print('  Robot at position. Press SPACE when you can see the tip in the camera.')
        _wait_for_space('Robot at point — SPACE to continue')

        # Move arm out of the way so we can see the table point
        _move(robot, 200, 0, Z_CLEAR)

        print('  Place a RED marker exactly where the tip was.')
        print('  SPACE to capture once the green dot appears on the marker.')

        detected = None
        while True:
            ret, frame = cam.read()
            frame = undistort(frame)
            center = detect_red_center(frame)

            display = frame.copy()
            if center:
                cv2.circle(display, center, 8, (0, 255, 0), -1)
                cv2.circle(display, center, 12, (0, 255, 0), 2)
                detected = center

            label = f'Point {i+1}/12  ({rx:.0f}, {ry:.0f}) mm  |  SPACE to capture'
            cv2.putText(display, label, (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
            if detected:
                cv2.putText(display, f'Detected: {detected}', (20, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)

            cv2.imshow('Calibration', display)
            key = cv2.waitKey(1) & 0xFF
            if key == 32 and detected is not None:
                print(f'  Captured pixel: {detected}')
                pixel_points.append(detected)
                break
            if key == ord('q'):
                print('Aborted.')
                cam.release()
                cv2.destroyAllWindows()
                sys.exit(0)

    return np.array(pixel_points, dtype=np.float32)


def _wait_for_space(label: str):
    while True:
        ret, frame = cam.read()
        display = undistort(frame)
        cv2.putText(display, label, (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        cv2.imshow('Calibration', display)
        if cv2.waitKey(1) & 0xFF == 32:
            break


# -----------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------

def main():
    print('Connecting to robot...')
    robot = Dobot(port=SERIAL_PORT, verbose=False)
    robot.speed(velocity=50, acceleration=50)
    print('Connected.')

    pixel_points = collect_calibration(robot)

    if len(pixel_points) < 4:
        print('Not enough points captured.')
        return

    H, _ = cv2.findHomography(pixel_points, ROBOT_POINTS, cv2.RANSAC)
    print('\nHomography matrix:')
    print(H)

    np.save(str(OUT_FILE), H)
    print(f'\nSaved to {OUT_FILE}')

    # Quick sanity check — re-project first point
    test_px = np.array([*pixel_points[0], 1.0])
    xy = H @ test_px
    xy /= xy[2]
    print(f'\nSanity check — pixel {pixel_points[0]} → robot ({xy[0]:.1f}, {xy[1]:.1f}) mm')
    print(f'Expected: ({ROBOT_POINTS[0][0]:.1f}, {ROBOT_POINTS[0][1]:.1f}) mm')

    cam.release()
    cv2.destroyAllWindows()
    robot.close()


if __name__ == '__main__':
    main()
