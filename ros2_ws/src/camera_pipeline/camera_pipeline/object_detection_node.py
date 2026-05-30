import os
import sys
from pathlib import Path

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from pick_interfaces.msg import ObjectDetection

# ---------------------------------------------------------------------------
# Path to provided_code/ — override with env var PROVIDED_CODE_PATH if needed
# ---------------------------------------------------------------------------
_default = Path(__file__).resolve().parents[5] / 'provided_code'
PROVIDED_CODE = Path(os.environ.get('PROVIDED_CODE_PATH', str(_default)))
sys.path.insert(0, str(PROVIDED_CODE))

# TODO: uncomment when provided_code deps are available
# from pickCVBlock import pixel_to_robot, phase_detect_plates, phase_detect_targets


class ObjectDetectionNode(Node):
    """
    Subscribes to both cameras, runs CV detection, and publishes ObjectDetection messages.

    Detection logic from provided_code/pickCVBlock.py is wired in via TODO markers below.
    """

    def __init__(self):
        super().__init__('object_detection_node')

        self._bridge = CvBridge()

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
    # Detection logic
    # -----------------------------------------------------------------------

    def _run_detection(self, frame, source: str):
        if self._H is None:
            return

        # TODO: call phase_detect_plates(frame) or phase_detect_targets(frame)
        #       from pickCVBlock to get pixel-space detections, then convert:
        #
        #   detections = phase_detect_targets()  # returns list of (rx, ry) in robot coords
        #   for (rx, ry) in detections:
        #       det = ObjectDetection()
        #       det.header.stamp = self.get_clock().now().to_msg()
        #       det.label = 'target'
        #       det.robot_x = rx
        #       det.robot_y = ry
        #       self._det_pub.publish(det)

        pass


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
