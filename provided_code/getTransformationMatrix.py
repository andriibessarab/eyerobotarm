import lib.DobotDllType as dType
import dobotArm
import time
import numpy as np
import cv2
import os

# Useful Global Variables
CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"
}

cam = cv2.VideoCapture(0)

if not cam.isOpened():
    print("Camera failed to open")
    exit()
    
#if the program errors for file path problems, copy the relative path to camera_params.npz and paste it here and try again. 
data = np.load("Collaborative_Robotics\camera_params.npz")
camera_matrix = data["camera_matrix"]
dist_coeffs   = data["dist_coeffs"]

# compute undistort maps once
ret,frame = cam.read()
h,w = frame.shape[:2]

new_K, roi = cv2.getOptimalNewCameraMatrix(
    camera_matrix,
    dist_coeffs,
    (w,h),
    1
)

map1, map2 = cv2.initUndistortRectifyMap(
    camera_matrix,
    dist_coeffs,
    None,
    new_K,
    (w,h),
    cv2.CV_16SC2
)

api = dType.load()

# robot coordinates in mm
robot_points = np.array([
    [200,-80],
    [230,-80],
    [260,-80],

    [200,-40],
    [230,-40],
    [260,-40],

    [200,0],
    [230,0],
    [260,0],

    [200,40],
    [230,40],
    [260,40]
], dtype=np.float32)


def detect_red_center(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower1 = np.array([0,120,70])
    upper1 = np.array([10,255,255])

    lower2 = np.array([170,120,70])
    upper2 = np.array([180,255,255])

    mask1 = cv2.inRange(hsv,lower1,upper1)
    mask2 = cv2.inRange(hsv,lower2,upper2)
    mask = mask1 + mask2

    mask = cv2.medianBlur(mask,5)

    contours,_ = cv2.findContours(mask,
                                  cv2.RETR_EXTERNAL,
                                  cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) == 0:
        return None

    c = max(contours,key=cv2.contourArea)

    M = cv2.moments(c)

    if M["m00"] == 0:
        return None

    cx = int(M["m10"]/M["m00"])
    cy = int(M["m01"]/M["m00"])

    return cx,cy


# ------------------------------------------------
# CALIBRATION
# ------------------------------------------------

def collect_calibration():

    pixel_points = []

    for pt in robot_points:

        x, y = pt

        print("\n----------------------------------")
        print("Moving robot to:", pt)

        # move to pick height
        dobotArm.move_to_xyz(api, x, y, -24)

        print("Press SPACE when robot is in position")
        
        # wait for space
        while True:
            ret, frame = cam.read()
            frame = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)

            cv2.putText(frame,
                        "Robot at point. Press SPACE.",
                        (30,40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0,255,0),
                        2)

            cv2.imshow("Calibration", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == 32:  # space
                break

        # move robot away so camera can see point
        print("Moving robot away")
        dobotArm.move_to_xyz(api, 200, 0, 80)

        print("Place RED marker where the tip was")
        print("Press SPACE to capture pixel")

        detected = None

        while True:

            ret, frame = cam.read()
            frame = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)

            center = detect_red_center(frame)

            if center is not None:
                u, v = center
                cv2.circle(frame, (u,v), 6, (0,255,0), -1)
                detected = center

            cv2.putText(frame,
                        "Place marker. SPACE to save",
                        (30,40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0,255,0),
                        2)

            cv2.imshow("Calibration", frame)

            key = cv2.waitKey(1) & 0xFF

            if key == 32 and detected is not None:
                print("Saved pixel:", detected)
                pixel_points.append(detected)
                break

    return np.array(pixel_points, dtype=np.float32)


def compute_homography(pixel_points):

    H,status = cv2.findHomography(pixel_points,robot_points)

    print("\nHomography Matrix\n")
    print(H)

    np.save("HomographyMatrix.npy",H)

    print("Matrix saved")

    return H


# ------------------------------------------------
# MAIN
# ------------------------------------------------

def run():
    dobotArm.initialize_robot(api)

    pixel_points = collect_calibration()

    if len(pixel_points) < 4:
        print("Not enough points")
        return

    compute_homography(pixel_points)

    cam.release()
    cv2.destroyAllWindows()


# Good Luck!
run()