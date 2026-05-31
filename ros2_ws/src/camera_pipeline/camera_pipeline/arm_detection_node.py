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
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision
from std_msgs.msg import Bool

_default = Path(__file__).resolve().parents[4] / 'scripts'
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

        model_path = str(PROVIDED_CODE / 'hand_landmarker.task')
        base_options = mp_python.BaseOptions(model_asset_path=model_path)
        options = mp_vision.HandLandmarkerOptions(
            base_options=base_options,
            num_hands=1,
            min_hand_detection_confidence=0.8,
            min_tracking_confidence=0.5,
            running_mode=mp_vision.RunningMode.VIDEO,
        )
        self.hands = mp_vision.HandLandmarker.create_from_options(options)
        self._frame_ts = 0

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
        self._pub_palm = self.create_publisher(Bool, '/workspace/palm', 10)
        self.get_logger().info('arm_detection_node ready')
    
    @staticmethod
    def get_hand_normal(hand):
        wrist = np.array([hand[0].x, hand[0].y, hand[0].z])
        index = np.array([hand[5].x, hand[5].y, hand[5].z])
        pinky = np.array([hand[17].x, hand[17].y, hand[17].z])

        v1 = index - wrist
        v2 = pinky - wrist

        normal = np.cross(v1, v2)
        normal = normal / (np.linalg.norm(normal) + 1e-6)

        return normal

    def _pixel_to_robot(self, u: float, v: float):
        p = np.array([u, v, 1.0])
        xy = self._H @ p
        xy /= xy[2]
        return float(xy[0]), float(xy[1])

    def _cb_frame(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        height, width = frame.shape[:2]

        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
        self._frame_ts += 33
        result = self.hands.detect_for_video(mp_image, self._frame_ts)

        if result.hand_landmarks:
            hand_landmarks = result.hand_landmarks[0]

            wrist      = hand_landmarks[0]
            middle_mcp = hand_landmarks[9]

            # Palm centre: midpoint of wrist and middle-finger MCP
            palm_x = ((wrist.x + middle_mcp.x) / 2) * width
            palm_y = ((wrist.y + middle_mcp.y) / 2) * height
            normal = self.get_hand_normal(hand_landmarks)

            if result.handedness[0][0].category_name == 'Left':
                normal = -normal  
            if (normal[2] < 0):  
                self.get_logger().info('Palm towards camera', throttle_duration_sec=0.5)
            else:
                self.get_logger().info(f'Palm away from camera', throttle_duration_sec=0.5)
            pixel_msg = Point()
            pixel_msg.x = palm_x
            pixel_msg.y = palm_y
            self._pub_pixel.publish(pixel_msg)

            if self._H is not None:
                robot_x, robot_y = self._pixel_to_robot(palm_x, palm_y)
                pos_msg = Point()
                pos_msg.x = robot_x
                pos_msg.y = robot_y
                pos_msg.z = 0.0
                self.get_logger().info(
                    f'hand: robot=({robot_x:.1f}, {robot_y:.1f}) mm',
                    throttle_duration_sec=1.0,
                )
                self._pub.publish(pos_msg)
                palm_msg = Bool()
                palm_msg.data = bool(normal[2] < 0)
                self._pub_palm.publish(palm_msg)
        else:
            self.get_logger().warn('No hand detected', throttle_duration_sec=2.0)


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
