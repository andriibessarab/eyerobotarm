import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import Image

from pick_interfaces.msg import ObjectDetection

# Colour and label config per detection type
_STYLE = {
    'target':   {'color': (0,   255,   0), 'label': 'TARGET'},
    'drop_zone':{'color': (0,   200, 255), 'label': 'DROP ZONE'},
}


class PreviewNode(Node):
    """Overlays detections on the workspace camera feed and shows a live window."""

    def __init__(self):
        super().__init__('preview_node')

        self._bridge = CvBridge()
        self._latest_frame = None
        self._latest_arm_position: Point | None = None
        # pending detections cleared each time we draw a new frame
        self._pending: list[ObjectDetection] = []
        self._display_scale = 0.2

        self.create_subscription(Image, '/workspace_camera/image_raw', self._cb_frame, 10)
        self.create_subscription(
            ObjectDetection,
            '/object_detection_node/detected_objects',
            self._cb_detection,
            50,
        )
        self.create_subscription(Point, '/workspace/arm_position', self._cb_arm_position, 10)

        self.create_timer(1.0 / 30.0, self._draw)
        self.get_logger().info('preview_node ready — press Q in the window to quit')

    def _cb_frame(self, msg: Image):
        self._latest_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._pending.clear()

    def _cb_detection(self, msg: ObjectDetection):
        self._pending.append(msg)

    def _cb_arm_position(self, msg: Point):
        self._latest_arm_position = msg

    def _draw(self):
        if self._latest_frame is None:
            return

        frame = self._latest_frame.copy()

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

        # HUD
        n_targets = sum(1 for d in self._pending if d.label == 'target')
        n_zones   = sum(1 for d in self._pending if d.label == 'drop_zone')
        cv2.putText(frame, f'targets: {n_targets}  drop zones: {n_zones}',
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, 'Q to quit', (10, frame.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (160, 160, 160), 1, cv2.LINE_AA)

        if self._latest_arm_position is not None:
            arm_text = f"arm_position: ({self._latest_arm_position.x:.0f}, {self._latest_arm_position.y:.0f})"
            cv2.circle(frame, (25, 55), 8, (0, 0, 255), -1)
            cv2.putText(frame, arm_text, (40, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2, cv2.LINE_AA)

        if self._display_scale != 1.0:
            frame = cv2.resize(frame, None, fx=self._display_scale, fy=self._display_scale, interpolation=cv2.INTER_AREA)

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
