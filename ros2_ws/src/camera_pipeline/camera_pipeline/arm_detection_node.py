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
import mediapipe as mp

_default = Path(__file__).resolve().parents[4] / 'provided_code'
PROVIDED_CODE = Path(os.environ.get('PROVIDED_CODE_PATH', str(_default)))
sys.path.insert(0, str(PROVIDED_CODE))


class ArmDetectionNode(Node):
    """
    Subscribes to the overhead workspace camera, detects the human hand using
    MediaPipe Hands, estimates the palm centre as the midpoint of wrist (0) and
    middle-finger MCP (9), converts to robot XY via the homography matrix, and
    publishes a geometry_msgs/Point. Only publishes when a hand is detected and
    the homography is loaded.
    """

    def __init__(self):
        super().__init__('arm_detection_node')

        self._bridge = CvBridge()
        self._map1 = None
        self._map2 = None

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.8,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils

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

        self._sub = self.create_subscription(
            Image, 'workspace_camera/image_raw', self._cb_frame, 10
        )
        self._pub       = self.create_publisher(Point, '/workspace/arm_position',       10)
        self._pub_pixel = self.create_publisher(Point, '/workspace/arm_position_pixel', 10)

        self.get_logger().info('arm_detection_node ready')

    def _pixel_to_robot(self, u: float, v: float):
        p = np.array([u, v, 1.0])
        xy = self._H @ p
        xy /= xy[2]
        return float(xy[0]), float(xy[1])

    def _cb_frame(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        height, width = frame.shape[:2]

        result = self.hands.process(rgb)

        if result.multi_hand_landmarks:
            hand_landmarks = result.multi_hand_landmarks[0]

            wrist      = hand_landmarks.landmark[0]
            middle_mcp = hand_landmarks.landmark[9]

            # Palm centre: midpoint of wrist and middle-finger MCP
            palm_x = ((wrist.x + middle_mcp.x) / 2) * width
            palm_y = ((wrist.y + middle_mcp.y) / 2) * height

            robot_x, robot_y = self._pixel_to_robot(palm_x, palm_y)
            confidence = getattr(wrist, 'visibility', 1.0)

            msg = Point()
            msg.x = robot_x
            msg.y = robot_y
            msg.z = 0.0

            self.get_logger().info(
                f'arm_position: robot=({robot_x:.1f}, {robot_y:.1f}) mm  confidence={confidence:.2f}',
                throttle_duration_sec=1.0,
            )
            self._pub.publish(msg)

            pixel_msg = Point()
            pixel_msg.x = palm_x
            pixel_msg.y = palm_y
            self._pub_pixel.publish(pixel_msg)
        else:
            self.get_logger().warn(
                'No hand detected in workspace_camera/image_raw',
                throttle_duration_sec=2.0,
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
