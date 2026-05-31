import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import Image

from pick_interfaces.msg import TagDetectionArray

# Keys
KEY_Q    = ord('q')
KEY_TAB  = 9   # Tab — toggle camera


class PreviewNode(Node):
    """Live preview window. Press Tab to toggle workspace/gaze camera. Press Q to quit."""

    def __init__(self):
        super().__init__('preview_node')

        self._bridge = CvBridge()
        self._workspace_frame = None
        self._gaze_frame = None
        self._latest_arm_pixel: Point | None = None
        self._latest_tags: TagDetectionArray | None = None
        self._display_scale = 0.5
        self._show_gaze = False  # False = workspace, True = gaze

        self.create_subscription(Image, '/workspace_camera/image_raw', self._cb_workspace, 10)
        self.create_subscription(Image, '/gaze_camera/image_raw',      self._cb_gaze,      10)
        self.create_subscription(Point, '/workspace/arm_position_pixel', self._cb_arm_pixel, 10)
        self.create_subscription(TagDetectionArray, '/workspace/tag_detections', self._cb_tags, 10)

        self.create_timer(1.0 / 30.0, self._draw)
        self.get_logger().info('preview_node ready — Tab: toggle camera | Q: quit')

    def _cb_workspace(self, msg: Image):
        self._workspace_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _cb_gaze(self, msg: Image):
        self._gaze_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _cb_arm_pixel(self, msg: Point):
        self._latest_arm_pixel = msg

    def _cb_tags(self, msg: TagDetectionArray):
        self._latest_tags = msg

    def _draw(self):
        frame_src = self._gaze_frame if self._show_gaze else self._workspace_frame
        if frame_src is None:
            return

        frame = frame_src.copy()
        s = self._display_scale

        # Resize first so overlay coordinates only need one scale factor
        if s != 1.0:
            frame = cv2.resize(frame, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)

        # Overlays only apply to the workspace camera
        if not self._show_gaze:
            if self._latest_tags is not None:
                for det in self._latest_tags.detections:
                    cx = int(det.pixel_x * s)
                    cy = int(det.pixel_y * s)
                    cv2.circle(frame, (cx, cy), 6, (255, 0, 0), -1)
                    cv2.putText(frame, str(det.tag_id), (cx + 8, cy - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 0), 1)

            if self._latest_arm_pixel is not None:
                arm_x = int(self._latest_arm_pixel.x * s)
                arm_y = int(self._latest_arm_pixel.y * s)
                cv2.circle(frame, (arm_x, arm_y), 8, (0, 0, 255), -1)

        # Camera label
        label = 'GAZE CAM' if self._show_gaze else 'WORKSPACE CAM'
        cv2.putText(frame, label, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)

        cv2.imshow('S26 Preview  [Tab: toggle | Q: quit]', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == KEY_Q:
            cv2.destroyAllWindows()
            rclpy.shutdown()
        elif key == KEY_TAB:
            self._show_gaze = not self._show_gaze
            self.get_logger().info(f"Switched to {'gaze' if self._show_gaze else 'workspace'} camera")

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
