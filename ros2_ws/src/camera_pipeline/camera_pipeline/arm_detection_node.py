import os
import sys
from pathlib import Path

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import Image

_default = Path(__file__).resolve().parents[4] / 'provided_code'
PROVIDED_CODE = Path(os.environ.get('PROVIDED_CODE_PATH', str(_default)))
sys.path.insert(0, str(PROVIDED_CODE))


class ArmDetectionNode(Node):
    """
    Detects a human hand in the overhead workspace camera using MediaPipe Hands.
    Converts the wrist pixel position to robot XY (mm) via the homography matrix
    and publishes a geometry_msgs/Point on /workspace/arm_position.

    Only publishes when a hand is confidently detected — the topic goes silent
    when no hand is in frame, so the task_coordinator falls back to its
    configured drop_fallback_x/y.
    """

    def __init__(self):
        super().__init__('arm_detection_node')

        self._bridge = CvBridge()
        self._map1 = None
        self._map2 = None

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

        # MediaPipe Hands — static_image_mode=False for video stream
        self._hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.5,
        )

        self._sub = self.create_subscription(
            Image, '/workspace_camera/image_raw', self._cb_frame, 10
        )
        self._pub = self.create_publisher(Point, '/workspace/arm_position', 10)

        self.get_logger().info('arm_detection_node ready')

    def _pixel_to_robot(self, u: float, v: float):
        p = np.array([u, v, 1.0])
        xy = self._H @ p
        xy /= xy[2]
        return float(xy[0]), float(xy[1])

    def _cb_frame(self, msg: Image):
        if self._H is None:
            return

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Build undistort maps once on first frame
        if self._map1 is None and self._camera_matrix is not None:
            h, w = frame.shape[:2]
            new_K, _ = cv2.getOptimalNewCameraMatrix(
                self._camera_matrix, self._dist_coeffs, (w, h), 1)
            self._map1, self._map2 = cv2.initUndistortRectifyMap(
                self._camera_matrix, self._dist_coeffs, None, new_K, (w, h), cv2.CV_16SC2)

        if self._map1 is not None:
            frame = cv2.remap(frame, self._map1, self._map2, cv2.INTER_LINEAR)

        # MediaPipe expects RGB
        results = self._hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        if not results.multi_hand_landmarks:
            return

        h, w = frame.shape[:2]
        # Use wrist landmark (index 0) — most stable under the overhead view
        wrist = results.multi_hand_landmarks[0].landmark[0]
        pixel_x = wrist.x * w
        pixel_y = wrist.y * h

        robot_x, robot_y = self._pixel_to_robot(pixel_x, pixel_y)

        pt = Point()
        pt.x, pt.y, pt.z = robot_x, robot_y, 0.0
        self._pub.publish(pt)

        self.get_logger().info(
            f'arm at pixel ({pixel_x:.0f}, {pixel_y:.0f}) → robot ({robot_x:.1f}, {robot_y:.1f}) mm',
            throttle_duration_sec=1.0,
        )

    def destroy_node(self):
        self._hands.close()
        super().destroy_node()


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
