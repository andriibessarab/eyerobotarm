import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class GazeCameraNode(Node):
    """Publishes frames from the glasses-mounted (gaze) camera.

    Two modes (mutually exclusive):
    - camera_source: device index string ('0', '1') or URL — opens VideoCapture
    - camera_topic:  if non-empty, relays an existing image topic instead
                     (useful for local testing: point at /workspace_camera/image_raw)

    camera_topic takes priority when set.
    """

    def __init__(self):
        super().__init__('gaze_camera_node')

        self.declare_parameter('camera_source', 'tcp://10.12.194.1:5000')
        self.declare_parameter('camera_topic', '')
        self.declare_parameter('flip_horizontal', False)
        self.declare_parameter('flip_vertical', False)

        src   = self.get_parameter('camera_source').get_parameter_value().string_value
        topic = self.get_parameter('camera_topic').get_parameter_value().string_value

        self._bridge = CvBridge()
        self._pub = self.create_publisher(Image, '/gaze_camera/image_raw', 10)
        self._cap = None

        if topic:
            self.create_subscription(Image, topic, self._relay, 10)
            self.get_logger().info(f'gaze_camera_node ready (relaying topic: {topic})')
        else:
            if src.isdigit():
                self._cap = cv2.VideoCapture(int(src))
            else:
                self._cap = cv2.VideoCapture(src, cv2.CAP_FFMPEG)
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not self._cap.isOpened():
                self.get_logger().warn(f'Could not open camera_source: {src}')
            self.create_timer(1.0 / 30.0, self._capture)
            self.get_logger().info(f'gaze_camera_node ready (camera_source={src})')

    def _relay(self, msg: Image):
        """Re-stamp and republish an incoming image on the gaze topic."""
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gaze_camera'
        self._pub.publish(msg)

    def _capture(self):
        ret, frame = self._cap.read()
        if not ret:
            self.get_logger().warn('gaze_camera: failed to read frame', throttle_duration_sec=5.0)
            return
        flip_h = self.get_parameter('flip_horizontal').value
        flip_v = self.get_parameter('flip_vertical').value
        if flip_h and flip_v:
            frame = cv2.flip(frame, -1)
        elif flip_h:
            frame = cv2.flip(frame, 1)
        elif flip_v:
            frame = cv2.flip(frame, 0)
        msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gaze_camera'
        self._pub.publish(msg)

    def destroy_node(self):
        if self._cap is not None:
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GazeCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
