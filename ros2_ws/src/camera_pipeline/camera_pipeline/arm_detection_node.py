import os
import sys
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import Image

_default = Path(__file__).resolve().parents[4] / 'provided_code'
PROVIDED_CODE = Path(os.environ.get('PROVIDED_CODE_PATH', str(_default)))
sys.path.insert(0, str(PROVIDED_CODE))

# Skin-tone HSV range (works under typical indoor lighting)
_SKIN_LOWER = np.array([0,  30,  60], dtype=np.uint8)
_SKIN_UPPER = np.array([25, 170, 255], dtype=np.uint8)
_MIN_AREA   = 4000   # px² — ignore small noise blobs


class ArmDetectionNode(Node):
    """
    Detects a human hand/arm in the overhead workspace camera using skin-colour
    HSV segmentation. Converts the largest skin-region centroid to robot XY (mm)
    via the homography matrix and publishes on /workspace/arm_position.

    Only publishes when a hand is detected — topic goes silent otherwise,
    so task_coordinator falls back to its drop_fallback param.
    """

    def __init__(self):
        super().__init__('arm_detection_node')

        self._bridge = CvBridge()
        self._map1   = None
        self._map2   = None
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))

        h_path   = PROVIDED_CODE / 'HomographyMatrix.npy'
        cam_path = PROVIDED_CODE / 'camera_params.npz'

        self._H             = None
        self._camera_matrix = None
        self._dist_coeffs   = None

        if h_path.exists():
            self._H = np.load(str(h_path))
            self.get_logger().info('Loaded HomographyMatrix.npy')
        else:
            self.get_logger().warn(f'HomographyMatrix.npy not found at {h_path}')

        if cam_path.exists():
            data = np.load(str(cam_path))
            self._camera_matrix = data['camera_matrix']
            self._dist_coeffs   = data['dist_coeffs']
            self.get_logger().info('Loaded camera_params.npz')
        else:
            self.get_logger().warn(f'camera_params.npz not found at {cam_path}')

        self._sub = self.create_subscription(
            Image, '/workspace_camera/image_raw', self._cb_frame, 10
        )
        self._pub = self.create_publisher(Point, '/workspace/arm_position', 10)

        self.get_logger().info('arm_detection_node ready (skin-HSV detection)')

    def _pixel_to_robot(self, u: float, v: float):
        p = np.array([u, v, 1.0])
        xy = self._H @ p
        xy /= xy[2]
        return float(xy[0]), float(xy[1])

    def _cb_frame(self, msg: Image):
        if self._H is None:
            return

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Lazy undistort map init
        if self._map1 is None and self._camera_matrix is not None:
            h, w = frame.shape[:2]
            new_K, _ = cv2.getOptimalNewCameraMatrix(
                self._camera_matrix, self._dist_coeffs, (w, h), 1)
            self._map1, self._map2 = cv2.initUndistortRectifyMap(
                self._camera_matrix, self._dist_coeffs, None, new_K, (w, h), cv2.CV_16SC2)

        if self._map1 is not None:
            frame = cv2.remap(frame, self._map1, self._map2, cv2.INTER_LINEAR)

        # Skin-colour segmentation
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, _SKIN_LOWER, _SKIN_UPPER)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self._kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < _MIN_AREA:
            return

        M = cv2.moments(largest)
        if M['m00'] == 0:
            return

        cx = M['m10'] / M['m00']
        cy = M['m01'] / M['m00']
        robot_x, robot_y = self._pixel_to_robot(cx, cy)

        pt = Point()
        pt.x, pt.y, pt.z = robot_x, robot_y, 0.0
        self._pub.publish(pt)

        self.get_logger().info(
            f'arm at pixel ({cx:.0f}, {cy:.0f}) → robot ({robot_x:.1f}, {robot_y:.1f}) mm',
            throttle_duration_sec=1.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArmDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
