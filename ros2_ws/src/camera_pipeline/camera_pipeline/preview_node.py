import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

from pick_interfaces.msg import TagDetectionArray

KEY_Q   = ord('q')
KEY_TAB = 9

LOCK_DISPLAY_SEC = 2.0  # how long to show "LOCKED" banner after a lock fires

# colours (BGR)
C_TAG    = (255,   0,   0)
C_ARM    = (  0,   0, 255)
C_TRACK  = (  0, 200, 255)   # yellow-ish: tag detected, accumulating
C_LOCK   = (  0, 220,   0)   # green: lock confirmed
C_LABEL  = (  0, 255, 255)
C_DIM    = (120, 120, 120)


class PreviewNode(Node):
    """
    Live preview window.
    - Tab : toggle between workspace camera (with overlays) and gaze camera
    - Q   : quit

    Gaze camera view shows:
    - Yellow banner while a tag is being tracked (accumulating stare time)
    - Green  banner for 2 s after a gaze lock fires
    """

    def __init__(self):
        super().__init__('preview_node')

        self._bridge = CvBridge()
        self._workspace_frame = None
        self._gaze_frame      = None
        self._latest_arm_pixel: Point | None = None
        self._latest_tags: TagDetectionArray | None = None
        self._display_scale = 0.5
        self._show_gaze     = False

        # Gaze state
        self._gaze_tracking_id  = -1   # current candidate from ~/tracking
        self._gaze_locked_id    = -1   # last confirmed lock
        self._gaze_lock_until   = 0.0  # time.time() until lock banner shows

        self.create_subscription(Image, '/workspace_camera/image_raw',    self._cb_workspace,  10)
        self.create_subscription(Image, '/gaze_camera/image_raw',          self._cb_gaze,       10)
        self.create_subscription(Point, '/workspace/arm_position_pixel',   self._cb_arm_pixel,  10)
        self.create_subscription(TagDetectionArray, '/workspace/tag_detections', self._cb_tags, 10)
        self.create_subscription(Int32, '/apriltag_gaze_node/tracking',    self._cb_tracking,   10)
        self.create_subscription(Int32, '/gaze/gazed_tag_id',              self._cb_locked,     10)

        self.create_timer(1.0 / 30.0, self._draw)
        self.get_logger().info('preview_node ready — Tab: toggle camera | Q: quit')

    # ── callbacks ──────────────────────────────────────────────────────────

    def _cb_workspace(self, msg: Image):
        self._workspace_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _cb_gaze(self, msg: Image):
        self._gaze_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _cb_arm_pixel(self, msg: Point):
        self._latest_arm_pixel = msg

    def _cb_tags(self, msg: TagDetectionArray):
        self._latest_tags = msg
        #self.get_logger().info(f'Length of April Tags: {len(msg.detections)}')

    def _cb_tracking(self, msg: Int32):
        self._gaze_tracking_id = msg.data

    def _cb_locked(self, msg: Int32):
        if msg.data >= 0:
            self._gaze_locked_id  = msg.data
            self._gaze_lock_until = time.time() + LOCK_DISPLAY_SEC

    # ── draw ───────────────────────────────────────────────────────────────

    def _draw(self):
        s = self._display_scale

        if self._show_gaze:
            frame = self._get_frame(self._gaze_frame, s)
            self._overlay_gaze(frame, s)
            cv2.putText(frame, 'GAZE CAM', (8, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, C_LABEL, 1)
        else:
            frame = self._get_frame(self._workspace_frame, s)
            self._overlay_workspace(frame, s)
            cv2.putText(frame, 'WORKSPACE CAM', (8, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, C_LABEL, 1)

        cv2.imshow('S26 Preview  [Tab: toggle | Q: quit]', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == KEY_Q:
            cv2.destroyAllWindows()
            rclpy.shutdown()
        elif key == KEY_TAB:
            self._show_gaze = not self._show_gaze
            self.get_logger().info(
                f"Switched to {'gaze' if self._show_gaze else 'workspace'} camera"
            )

    def _get_frame(self, src, s):
        """Return a scaled copy of src, or a black placeholder if src is None."""
        if src is None:
            h, w = int(480 * s), int(640 * s)
            blank = np.zeros((h, w, 3), dtype=np.uint8)
            cv2.putText(blank, 'Waiting for camera...', (20, h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, C_DIM, 1)
            return blank
        frame = src.copy()
        if s != 1.0:
            frame = cv2.resize(frame, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
        return frame

    def _overlay_workspace(self, frame, s):
        if self._latest_tags is not None:
            for det in self._latest_tags.detections:
                cx = int(det.pixel_x * s)
                cy = int(det.pixel_y * s)
                cv2.circle(frame, (cx, cy), 6, C_TAG, -1)
                cv2.putText(frame, str(det.tag_id), (cx + 8, cy - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, C_TAG, 1)

        if self._latest_arm_pixel is not None:
            arm_x = int(self._latest_arm_pixel.x * s)
            arm_y = int(self._latest_arm_pixel.y * s)
            cv2.circle(frame, (arm_x, arm_y), 8, C_ARM, -1)

    def _overlay_gaze(self, frame, s):
        h, w = frame.shape[:2]
        now  = time.time()

        if now < self._gaze_lock_until:
            # Lock banner — green
            text   = f'LOCKED  tag {self._gaze_locked_id}'
            colour = C_LOCK
            thick  = 2
            scale  = 1.0
        elif self._gaze_tracking_id >= 0:
            # Accumulating — yellow
            text   = f'Detecting  tag {self._gaze_tracking_id}...'
            colour = C_TRACK
            thick  = 1
            scale  = 0.75
        else:
            # Nothing
            text   = 'No tag in view'
            colour = C_DIM
            thick  = 1
            scale  = 0.55

        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thick)
        tx = (w - tw) // 2
        ty = h - 20

        # Semi-transparent backing strip
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, ty - th - 8), (w, ty + 8), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)

        cv2.putText(frame, text, (tx, ty),
                    cv2.FONT_HERSHEY_SIMPLEX, scale, colour, thick, cv2.LINE_AA)

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
