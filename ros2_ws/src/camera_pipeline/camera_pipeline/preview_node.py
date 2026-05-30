import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import Image

from pick_interfaces.msg import ObjectDetection

_STYLE = {
    'target':   {'color': (0,   255,   0), 'label': 'TARGET'},
    'drop_zone':{'color': (0,   200, 255), 'label': 'DROP ZONE'},
}

_ARM_COLOR   = (0, 100, 255)   # orange — arm/hand marker
_ARM_TIMEOUT = 1.0             # seconds before arm marker fades


class PreviewNode(Node):
    """Overlays object detections and arm position on the workspace camera feed."""

    def __init__(self):
        super().__init__('preview_node')

        self._bridge        = CvBridge()
        self._latest_frame  = None
        self._pending: list[ObjectDetection] = []
        self._arm_pt: Point | None = None
        self._arm_stamp     = 0.0

        self.create_subscription(Image, '/workspace_camera/image_raw', self._cb_frame, 10)
        self.create_subscription(
            ObjectDetection, '/object_detection_node/detected_objects', self._cb_detection, 50)
        self.create_subscription(Point, '/workspace/arm_position', self._cb_arm, 10)

        self.create_timer(1.0 / 30.0, self._draw)
        self.get_logger().info('preview_node ready — press Q to quit')

    def _cb_frame(self, msg: Image):
        self._latest_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._pending.clear()

    def _cb_detection(self, msg: ObjectDetection):
        self._pending.append(msg)

    def _cb_arm(self, msg: Point):
        self._arm_pt    = msg
        self._arm_stamp = self.get_clock().now().nanoseconds / 1e9

    def _draw(self):
        if self._latest_frame is None:
            return

        frame = self._latest_frame.copy()

        # Object detections
        for det in self._pending:
            style = _STYLE.get(det.label, {'color': (255, 255, 255), 'label': det.label})
            color = style['color']
            cx, cy = int(det.pixel_x), int(det.pixel_y)
            if det.label == 'drop_zone':
                cv2.circle(frame, (cx, cy), 35, color, 2)
            else:
                cv2.rectangle(frame, (cx - 20, cy - 20), (cx + 20, cy + 20), color, 2)
            text = f"{style['label']}  ({det.robot_x:.0f}, {det.robot_y:.0f}) mm"
            cv2.putText(frame, text, (cx + 25, cy + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
            cv2.circle(frame, (cx, cy), 4, color, -1)

        # Arm position — shown as crosshair, fades after _ARM_TIMEOUT seconds
        now = self.get_clock().now().nanoseconds / 1e9
        if self._arm_pt is not None and (now - self._arm_stamp) < _ARM_TIMEOUT:
            # Back-project robot XY to pixel using the inverse is unavailable here,
            # so we label it in the corner with the robot coords
            rx, ry = self._arm_pt.x, self._arm_pt.y
            label = f'ARM  ({rx:.0f}, {ry:.0f}) mm'
            cv2.putText(frame, label,
                        (10, frame.shape[0] - 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, _ARM_COLOR, 2, cv2.LINE_AA)
            cv2.rectangle(frame,
                          (5, frame.shape[0] - 55),
                          (len(label) * 11 + 10, frame.shape[0] - 20),
                          _ARM_COLOR, 1)

        # HUD
        n_targets = sum(1 for d in self._pending if d.label == 'target')
        cv2.putText(frame, f'targets: {n_targets}',
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, 'Q to quit', (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1, cv2.LINE_AA)

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
