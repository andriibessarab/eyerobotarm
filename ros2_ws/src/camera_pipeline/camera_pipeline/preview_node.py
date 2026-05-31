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

from pick_interfaces.msg import TagDetectionArray

_default = Path(__file__).resolve().parents[4] / 'provided_code'
PROVIDED_CODE = Path(os.environ.get('PROVIDED_CODE_PATH', str(_default)))
sys.path.insert(0, str(PROVIDED_CODE))


class PreviewNode(Node):
    """Overlays detections on the workspace camera feed and shows a live window."""

    def __init__(self):
        super().__init__('preview_node')

        self._bridge = CvBridge()
        self._latest_frame = None
        self._latest_arm_position: Point | None = None
        self._latest_tags: TagDetectionArray | None = None
        self._display_scale = 0.5

        # Load inverse homography so robot mm coordinates can be drawn as pixels
        h_path = PROVIDED_CODE / 'HomographyMatrix.npy'
        self._H_inv = None
        if h_path.exists():
            H = np.load(str(h_path))
            self._H_inv = np.linalg.inv(H)
        else:
            self.get_logger().warn(f'HomographyMatrix.npy not found — arm position will not overlay correctly')

        self.create_subscription(Image, '/workspace_camera/image_raw', self._cb_frame, 10)
        self.create_subscription(Point, '/workspace/arm_position', self._cb_arm_position, 10)
        self.create_subscription(TagDetectionArray, '/workspace/tag_detections', self._cb_tags, 10)

        self.create_timer(1.0 / 30.0, self._draw)
        self.get_logger().info('preview_node ready — press Q in the window to quit')

    def _robot_to_pixel(self, robot_x: float, robot_y: float):
        """Convert robot mm coordinates back to image pixel coordinates."""
        p = self._H_inv @ np.array([robot_x, robot_y, 1.0])
        p /= p[2]
        return float(p[0]), float(p[1])

    def _cb_frame(self, msg: Image):
        self._latest_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _cb_arm_position(self, msg: Point):
        self._latest_arm_position = msg

    def _cb_tags(self, msg: TagDetectionArray):
        self._latest_tags = msg

    def _draw(self):
        if self._latest_frame is None:
            return

        frame = self._latest_frame.copy()
        s = self._display_scale

        if self._latest_tags is not None:
            for det in self._latest_tags.detections:
                cx = int(det.pixel_x * s)
                cy = int(det.pixel_y * s)
                cv2.circle(frame, (cx, cy), 6, (255, 0, 0), -1)
                cv2.putText(frame, str(det.tag_id), (cx + 8, cy - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 0), 1)

        if self._latest_arm_position is not None and self._H_inv is not None:
            px, py = self._robot_to_pixel(
                self._latest_arm_position.x, self._latest_arm_position.y
            )
            arm_x = int(px * s)
            arm_y = int(py * s)
            cv2.circle(frame, (arm_x, arm_y), 8, (0, 0, 255), -1)

        if s != 1.0:
            frame = cv2.resize(frame, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)

        cv2.imshow('S26 Workspace Camera', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PreviewNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
