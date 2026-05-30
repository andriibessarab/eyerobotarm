import os
import sys
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from pick_interfaces.msg import ObjectDetection

# ---------------------------------------------------------------------------
# Path to provided_code/ — override with env var PROVIDED_CODE_PATH if needed
# ---------------------------------------------------------------------------
_default = Path(__file__).resolve().parents[4] / 'provided_code'
PROVIDED_CODE = Path(os.environ.get('PROVIDED_CODE_PATH', str(_default)))
sys.path.insert(0, str(PROVIDED_CODE))


class ObjectDetectionNode(Node):
    """
    Subscribes to the workspace camera, runs CV detection, and publishes
    ObjectDetection messages for drop zones (Hough circles) and targets (red HSV).

    Detection algorithms are ported directly from provided_code/pickCVBlock.py.
    """

    def __init__(self):
        super().__init__('object_detection_node')

        self._bridge = CvBridge()

        # Lazy undistort maps — computed on first frame once dimensions are known
        self._map1 = None
        self._map2 = None

        # Load calibration files from provided_code/
        h_path = PROVIDED_CODE / 'HomographyMatrix.npy'
        cam_path = PROVIDED_CODE / 'camera_params.npz'

        self._H = None
        self._camera_matrix = None
        self._dist_coeffs = None

        if h_path.exists():
            self._H = np.load(str(h_path))
            self.get_logger().info('Loaded HomographyMatrix.npy')
        else:
            self.get_logger().warn(f'HomographyMatrix.npy not found at {h_path}')

        if cam_path.exists():
            data = np.load(str(cam_path))
            self._camera_matrix = data['camera_matrix']
            self._dist_coeffs = data['dist_coeffs']
            self.get_logger().info('Loaded camera_params.npz')
        else:
            self.get_logger().warn(f'camera_params.npz not found at {cam_path}')

        # Subscribers
        self._workspace_sub = self.create_subscription(
            Image, '/workspace_camera/image_raw', self._cb_workspace, 10
        )
        self._gaze_sub = self.create_subscription(
            Image, '/gaze_camera/image_raw', self._cb_gaze, 10
        )

        # Publisher
        self._det_pub = self.create_publisher(ObjectDetection, '~/detected_objects', 10)

        self.get_logger().info('object_detection_node ready')

    # -----------------------------------------------------------------------
    # Camera callbacks
    # -----------------------------------------------------------------------

    def _cb_workspace(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._run_detection(frame, source='workspace')

    def _cb_gaze(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._run_detection(frame, source='gaze')

    # -----------------------------------------------------------------------
    # Detection pipeline
    # -----------------------------------------------------------------------

    def _run_detection(self, frame, source: str):
        if self._H is None:
            return

        # Build undistort maps once on first frame (needs frame dimensions)
        if self._map1 is None and self._camera_matrix is not None:
            h, w = frame.shape[:2]
            new_K, _ = cv2.getOptimalNewCameraMatrix(
                self._camera_matrix, self._dist_coeffs, (w, h), 1)
            self._map1, self._map2 = cv2.initUndistortRectifyMap(
                self._camera_matrix, self._dist_coeffs, None, new_K, (w, h), cv2.CV_16SC2)

        if self._map1 is not None:
            frame = cv2.remap(frame, self._map1, self._map2, cv2.INTER_LINEAR)

        now = self.get_clock().now().to_msg()

        for px, py, rx, ry in self._detect_plates(frame):
            det = ObjectDetection()
            det.header.stamp = now
            det.label = 'drop_zone'
            det.pixel_x, det.pixel_y = px, py
            det.robot_x, det.robot_y = rx, ry
            det.confidence = 1.0
            self._det_pub.publish(det)

        for px, py, rx, ry in self._detect_targets(frame):
            det = ObjectDetection()
            det.header.stamp = now
            det.label = 'target'
            det.pixel_x, det.pixel_y = px, py
            det.robot_x, det.robot_y = rx, ry
            det.confidence = 1.0
            self._det_pub.publish(det)

    # -----------------------------------------------------------------------
    # CV helpers — ported from provided_code/pickCVBlock.py
    # -----------------------------------------------------------------------

    def _pixel_to_robot(self, u: float, v: float):
        """Convert pixel coords to robot XY (mm) using the loaded homography."""
        p = np.array([u, v, 1.0])
        xy = self._H @ p
        xy /= xy[2]
        return float(xy[0]), float(xy[1])

    def _detect_plates(self, frame) -> list:
        """Detect circular drop zones via Hough circles (from pickCVBlock.phase_detect_plates)."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.medianBlur(gray, 7)
        circles = cv2.HoughCircles(
            blurred, cv2.HOUGH_GRADIENT, 1, 150,
            param1=100, param2=35, minRadius=25, maxRadius=55)
        results = []
        if circles is not None:
            for c in np.uint16(np.around(circles))[0]:
                rx, ry = self._pixel_to_robot(float(c[0]), float(c[1]))
                results.append((float(c[0]), float(c[1]), rx, ry))
        return results

    def _detect_targets(self, frame) -> list:
        """Detect red velcro targets via HSV segmentation (from pickCVBlock.phase_detect_targets)."""
        hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (3, 3), 0), cv2.COLOR_BGR2HSV)
        mask = (
            cv2.inRange(hsv, np.array([0,   120, 70]), np.array([10,  255, 255])) |
            cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        results = []
        for cnt in contours:
            if cv2.contourArea(cnt) > 800:
                M = cv2.moments(cnt)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    rx, ry = self._pixel_to_robot(float(cx), float(cy))
                    results.append((float(cx), float(cy), rx, ry))
        return results


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
