import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import Image

from pick_interfaces.msg import TagDetectionArray


class PreviewNode(Node):
    """Overlays detections on the workspace camera feed and shows a live window."""

    def __init__(self):
        super().__init__('preview_node')

        self._bridge = CvBridge()
        self._latest_frame = None
        self._latest_arm_position: Point | None = None
        self._latest_tags: TagDetectionArray | None = None
        self._display_scale = 0.5

        self.create_subscription(Image, '/workspace_camera/image_raw', self._cb_frame, 10)
        self.create_subscription(Point, '/workspace/arm_position', self._cb_arm_position, 10)
        self.create_subscription(TagDetectionArray, '/workspace/tag_detections', self._cb_tags, 10)

        self.create_timer(1.0 / 30.0, self._draw)
        self.get_logger().info('preview_node ready — press Q in the window to quit')

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

        if self._latest_arm_position is not None:
            # arm_position is in robot mm — not directly drawable as pixels;
            # draw at stored pixel coords only if scale yields something sensible
            arm_x = int(self._latest_arm_position.x)
            arm_y = int(self._latest_arm_position.y)
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
