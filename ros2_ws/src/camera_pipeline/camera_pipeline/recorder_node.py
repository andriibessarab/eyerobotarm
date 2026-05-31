import datetime
import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

RECORDINGS_DIR = os.path.expanduser('~/recordings')


class RecorderNode(Node):
    """
    Subscribes to an image topic and writes every frame to an .mp4 file.
    Parameters:
      image_topic  (string) — topic to record, e.g. /workspace_camera/image_raw
      fps          (double) — output frame rate, default 30.0
      label        (string) — prefix for the filename, e.g. 'workspace' or 'gaze'
    """

    def __init__(self):
        super().__init__('recorder_node')

        self.declare_parameter('image_topic', '/workspace_camera/image_raw')
        self.declare_parameter('fps',         30.0)
        self.declare_parameter('label',       'camera')

        topic = self.get_parameter('image_topic').value
        fps   = float(self.get_parameter('fps').value)
        label = self.get_parameter('label').value

        os.makedirs(RECORDINGS_DIR, exist_ok=True)
        ts = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self._path = os.path.join(RECORDINGS_DIR, f'{ts}_{label}.mp4')

        self._bridge  = CvBridge()
        self._writer: cv2.VideoWriter | None = None
        self._fps     = fps
        self._fourcc  = cv2.VideoWriter_fourcc(*'mp4v')

        self.create_subscription(Image, topic, self._cb_frame, 10)
        self.get_logger().info(f'recorder_node: {topic} → {self._path}')

    def _cb_frame(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self._writer is None:
            h, w = frame.shape[:2]
            self._writer = cv2.VideoWriter(self._path, self._fourcc, self._fps, (w, h))
            self.get_logger().info(f'Recording started ({w}x{h} @ {self._fps}fps)')

        self._writer.write(frame)

    def destroy_node(self):
        if self._writer is not None:
            self._writer.release()
            self.get_logger().info(f'Recording saved → {self._path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RecorderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
