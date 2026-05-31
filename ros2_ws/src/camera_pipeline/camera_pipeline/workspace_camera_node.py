import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class WorkspaceCameraNode(Node):
    """Publishes frames from the overhead Orbbec camera.

    camera_source accepts a device index string ('0', '1', ...) or a URL.
    """

    def __init__(self):
        super().__init__('workspace_camera_node')

        self.declare_parameter('camera_source', '0')
        src = self.get_parameter('camera_source').get_parameter_value().string_value

        self._bridge = CvBridge()
        self._pub = self.create_publisher(Image, '/workspace_camera/image_raw', 10)

        self._cap = cv2.VideoCapture(int(src) if src.isdigit() else src)
        if not self._cap.isOpened():
            self.get_logger().warn(f'Could not open camera_source: {src}')

        self._timer = self.create_timer(1.0 / 30.0, self._capture)  # 30 Hz

        self.get_logger().info(f'workspace_camera_node ready (camera_source={src})')

    def _capture(self):
        ret, frame = self._cap.read()
        if not ret:
            self.get_logger().warn('workspace_camera: failed to read frame', throttle_duration_sec=5.0)
            return
        msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'workspace_camera'
        self._pub.publish(msg)

    def destroy_node(self):
        self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WorkspaceCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
