import os
import sys
from pathlib import Path

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from pick_interfaces.msg import TagDetection, TagDetectionArray

_default = Path(__file__).resolve().parents[4] / 'provided_code'
PROVIDED_CODE = Path(os.environ.get('PROVIDED_CODE_PATH', str(_default)))
sys.path.insert(0, str(PROVIDED_CODE))


class AprilTagWorkspaceNode(Node):
    """
    Subscribes to the overhead workspace camera, detects all AprilTags in each
    frame, converts their pixel centres to robot XY via the homography matrix,
    and publishes a TagDetectionArray.

    TODO: implement detection loop in _run_detection
      1. undistort frame (lazy — build cv2.initUndistortRectifyMap on first frame)
      2. cv2.aruco.detectMarkers(frame, DICT_APRILTAG_36H11)
      3. for each marker: pixel_to_robot → build TagDetection
      4. publish TagDetectionArray
    """

    def __init__(self):
        super().__init__('apriltag_workspace_node')

        self._bridge = CvBridge()
        self._map1 = None
        self._map2 = None

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
            Image, '/workspace_camera/image_raw', self._cb_frame, 10
        )
        self._pub = self.create_publisher(TagDetectionArray, '/workspace/tag_detections', 10)

        self.get_logger().info('apriltag_workspace_node ready (TODO: detection not yet implemented)')

    def _pixel_to_robot(self, u: float, v: float):
        p = np.array([u, v, 1.0])
        xy = self._H @ p
        xy /= xy[2]
        return float(xy[0]), float(xy[1])

    def _cb_frame(self, msg: Image):
        if self._H is None:
            return
        # TODO: implement detection and publish TagDetectionArray
        pass


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagWorkspaceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
