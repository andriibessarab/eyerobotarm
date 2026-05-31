import os
import sys
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from pupil_apriltags import Detector

from pick_interfaces.msg import TagDetection, TagDetectionArray

_default = Path(__file__).resolve().parents[4] / 'provided_code'
PROVIDED_CODE = Path(os.environ.get('PROVIDED_CODE_PATH', str(_default)))
sys.path.insert(0, str(PROVIDED_CODE))


class AprilTagWorkspaceNode(Node):
    """
    Subscribes to the overhead workspace camera, detects all AprilTags in each
    frame, converts their pixel centres to robot XY via the homography matrix,
    and publishes a pick_interfaces/TagDetectionArray.
    """

    def __init__(self):
        super().__init__('apriltag_workspace_node')

        self._bridge = CvBridge()

        # Must match the physical tags; pupil_apriltags supports families like tag36h11.
        self.declare_parameter('april_tag_family',   'tag36h11')
        self.declare_parameter('min_decision_margin', 70.0)   # raised from 20 — cuts false positives
        self.declare_parameter('quad_decimate',       1.0)    # 1.0 = full res; default 2.0 kills small tags
        self.declare_parameter('decode_sharpening',   0.8)    # higher helps decode small patches

        self._april_tag_family    = self.get_parameter('april_tag_family').value
        self._min_decision_margin = float(self.get_parameter('min_decision_margin').value)

        self.detector = Detector(
            families=self._april_tag_family,
            quad_decimate=float(self.get_parameter('quad_decimate').value),
            decode_sharpening=float(self.get_parameter('decode_sharpening').value),
        )
        self.get_logger().info(
            f'AprilTag detector: family={self._april_tag_family} '
            f'quad_decimate={self.get_parameter("quad_decimate").value} '
            f'min_decision_margin={self._min_decision_margin:.0f}'
        )

        h_path   = PROVIDED_CODE / 'HomographyMatrix.npy'
        cam_path = PROVIDED_CODE / 'camera_params.npz'

        self._H              = None
        self._camera_matrix  = None
        self._dist_coeffs    = None
        self._map1 = self._map2 = None

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
        self._pub = self.create_publisher(TagDetectionArray, '/workspace/tag_detections', 10)

        self.get_logger().info('apriltag_workspace_node ready')

    def _pixel_to_robot(self, u: float, v: float):
        p = np.array([u, v, 1.0])
        xy = self._H @ p
        xy /= xy[2]
        return float(xy[0]), float(xy[1])

    def _cb_frame(self, msg: Image):
        array_msg = TagDetectionArray()

        if self._H is None:
            self._pub.publish(array_msg)
            return

        gray = cv2.cvtColor(self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'), cv2.COLOR_BGR2GRAY)

        tags = self.detector.detect(gray)
        self.get_logger().info(
            f'AprilTag detections in frame: {len(tags)}',
            throttle_duration_sec=2.0,
        )

        for tag in tags:
       

            robot_x, robot_y = self._pixel_to_robot(float(tag.center[0]), float(tag.center[1]))
           
            t = TagDetection()
            t.tag_id  = int(tag.tag_id)
            t.pixel_x = float(tag.center[0])
            t.pixel_y = float(tag.center[1])
            t.robot_x = robot_x
            t.robot_y = robot_y
            array_msg.detections.append(t)

        self._pub.publish(array_msg)


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
