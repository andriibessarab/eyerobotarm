import time
from collections import deque
from datetime import datetime

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String

from pick_interfaces.msg import TagDetectionArray

# ── Layout ──────────────────────────────────────────────────────────────────
CAM_W, CAM_H   = 480, 360   # camera panel
PANEL_W        = 300         # status panel width
WIN_W          = CAM_W + PANEL_W
WIN_H          = CAM_H + 30  # +30 for top bar

FONT  = cv2.FONT_HERSHEY_SIMPLEX
FONT2 = cv2.FONT_HERSHEY_DUPLEX

# ── Colours (BGR) ────────────────────────────────────────────────────────────
BG        = (22,  22,  22)
PANEL_BG  = (32,  32,  32)
BORDER    = (55,  55,  55)
C_WHITE   = (220, 220, 220)
C_DIM     = (100, 100, 100)
C_IDLE    = ( 80, 200,  80)
C_EXEC    = ( 60, 200, 240)
C_WARN    = ( 60, 140, 255)
C_TAG     = (255, 100,  60)
C_ARM     = ( 60,  60, 255)
C_TRACK   = (  0, 200, 255)
C_LOCK    = (  0, 220,   0)
C_LABEL   = (  0, 255, 255)

KEY_Q   = ord('q')
KEY_TAB = 9
KEY_H   = ord('h')
KEY_V   = ord('v')

LOCK_DISPLAY_SEC = 2.5
MAX_LOG = 7


class PreviewNode(Node):

    def __init__(self):
        super().__init__('preview_node')

        self._bridge = CvBridge()

        # Camera state
        self._workspace_frame = None
        self._gaze_frame      = None
        self._show_gaze       = False
        self._flip_h          = False
        self._flip_v          = False

        # Detection state
        self._latest_tags: TagDetectionArray | None = None
        self._latest_arm_pixel: Point | None = None

        # Gaze state
        self._gaze_tracking_id = -1
        self._gaze_locked_id   = -1
        self._gaze_lock_until  = 0.0

        # Coordinator state
        self._coord_state  = 'WAITING'
        self._coord_action = '—'
        self._log: deque = deque(maxlen=MAX_LOG)

        # Subscriptions
        self.create_subscription(Image, '/workspace_camera/image_raw',       self._cb_workspace, 10)
        self.create_subscription(Image, '/gaze_camera/image_raw',             self._cb_gaze,      10)
        self.create_subscription(Point, '/workspace/arm_position_pixel',      self._cb_arm_pixel, 10)
        self.create_subscription(TagDetectionArray, '/workspace/tag_detections', self._cb_tags,   10)
        self.create_subscription(Int32, '/apriltag_gaze/tracking',            self._cb_tracking,  10)
        self.create_subscription(Int32, '/gaze/gazed_tag_id',                 self._cb_locked,    10)
        self.create_subscription(String, '/coordinator/state',                self._cb_state,     10)
        self.create_subscription(String, '/coordinator/status',               self._cb_status,    10)

        self.create_timer(1.0 / 30.0, self._draw)
        self.get_logger().info('preview_node ready  [Tab] cam  [H/V] flip  [Q] quit')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_workspace(self, msg):
        self._workspace_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _cb_gaze(self, msg):
        self._gaze_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _cb_arm_pixel(self, msg):
        self._latest_arm_pixel = msg

    def _cb_tags(self, msg):
        self._latest_tags = msg

    def _cb_tracking(self, msg):
        self._gaze_tracking_id = msg.data

    def _cb_locked(self, msg):
        if msg.data >= 0:
            self._gaze_locked_id  = msg.data
            self._gaze_lock_until = time.time() + LOCK_DISPLAY_SEC

    def _cb_state(self, msg):
        self._coord_state = msg.data

    def _cb_status(self, msg):
        self._coord_action = msg.data
        ts = datetime.now().strftime('%H:%M:%S')
        self._log.appendleft(f'{ts}  {msg.data}')

    # ── Draw ──────────────────────────────────────────────────────────────────

    def _draw(self):
        canvas = np.full((WIN_H, WIN_W, 3), BG, dtype=np.uint8)

        # ── Top bar ───────────────────────────────────────────────────────
        cv2.rectangle(canvas, (0, 0), (WIN_W, 28), PANEL_BG, -1)
        cv2.line(canvas, (0, 28), (WIN_W, 28), BORDER, 1)
        cv2.putText(canvas, 'S26  TOYOTA INNOVATION CHALLENGE', (10, 19),
                    FONT, 0.48, C_DIM, 1, cv2.LINE_AA)
        hint = '[Tab] cam  [H/V] flip  [Q] quit'
        tw = cv2.getTextSize(hint, FONT, 0.38, 1)[0][0]
        cv2.putText(canvas, hint, (WIN_W - tw - 8, 19),
                    FONT, 0.38, C_DIM, 1, cv2.LINE_AA)

        # ── Camera panel (left) ───────────────────────────────────────────
        src = self._gaze_frame if self._show_gaze else self._workspace_frame
        if src is not None:
            cam = cv2.resize(src.copy(), (CAM_W, CAM_H), interpolation=cv2.INTER_AREA)
            if self._show_gaze:
                if self._flip_h and self._flip_v: cam = cv2.flip(cam, -1)
                elif self._flip_h:               cam = cv2.flip(cam, 1)
                elif self._flip_v:               cam = cv2.flip(cam, 0)
        else:
            cam = np.zeros((CAM_H, CAM_W, 3), dtype=np.uint8)
            msg = 'Waiting for camera...'
            tw = cv2.getTextSize(msg, FONT, 0.55, 1)[0][0]
            cv2.putText(cam, msg, ((CAM_W - tw) // 2, CAM_H // 2),
                        FONT, 0.55, C_DIM, 1, cv2.LINE_AA)

        # Draw overlays on camera
        sx = CAM_W / 640.0
        sy = CAM_H / 480.0
        if not self._show_gaze:
            if self._latest_tags:
                for det in self._latest_tags.detections:
                    cx, cy = int(det.pixel_x * sx), int(det.pixel_y * sy)
                    cv2.circle(cam, (cx, cy), 7, C_TAG, -1)
                    cv2.circle(cam, (cx, cy), 7, (255,255,255), 1)
                    cv2.putText(cam, f'#{det.tag_id}', (cx + 9, cy - 5),
                                FONT, 0.42, C_TAG, 1, cv2.LINE_AA)
            if self._latest_arm_pixel:
                ax, ay = int(self._latest_arm_pixel.x * sx), int(self._latest_arm_pixel.y * sy)
                cv2.circle(cam, (ax, ay), 10, C_ARM, 2)
                cv2.circle(cam, (ax, ay),  3, C_ARM, -1)
        else:
            self._draw_gaze_overlay(cam)

        # Camera label
        cam_label = ('GAZE' if self._show_gaze else 'WORKSPACE') + ' CAM'
        cv2.putText(cam, cam_label, (8, 20), FONT, 0.5, C_LABEL, 1, cv2.LINE_AA)

        canvas[30:30+CAM_H, 0:CAM_W] = cam
        cv2.line(canvas, (CAM_W, 28), (CAM_W, WIN_H), BORDER, 1)

        # ── Status panel (right) ──────────────────────────────────────────
        px = CAM_W + 12  # panel x start
        py = 38          # panel y start

        # State badge
        state_col = C_IDLE if self._coord_state == 'IDLE' else (
                    C_EXEC if self._coord_state == 'EXECUTING' else C_WARN)
        badge_txt = self._coord_state
        btw = cv2.getTextSize(badge_txt, FONT2, 0.65, 1)[0][0]
        cv2.rectangle(canvas, (px, py), (px + btw + 16, py + 26),
                      tuple(c // 5 for c in state_col), -1)
        cv2.rectangle(canvas, (px, py), (px + btw + 16, py + 26), state_col, 1)
        cv2.putText(canvas, badge_txt, (px + 8, py + 18),
                    FONT2, 0.65, state_col, 1, cv2.LINE_AA)
        py += 36

        # Current action
        cv2.putText(canvas, 'CURRENT ACTION', (px, py), FONT, 0.36, C_DIM, 1, cv2.LINE_AA)
        py += 16
        # Word-wrap current action
        words = self._coord_action.split()
        lines, cur = [], ''
        for w in words:
            candidate = (cur + ' ' + w).strip()
            if cv2.getTextSize(candidate, FONT, 0.5, 1)[0][0] < PANEL_W - 20:
                cur = candidate
            else:
                lines.append(cur); cur = w
        if cur: lines.append(cur)
        for line in lines[:2]:
            cv2.putText(canvas, line, (px, py), FONT, 0.5, C_WHITE, 1, cv2.LINE_AA)
            py += 20
        py += 4

        # Separator
        cv2.line(canvas, (px, py), (WIN_W - 8, py), BORDER, 1)
        py += 10

        # Gaze section
        cv2.putText(canvas, 'GAZE', (px, py), FONT, 0.36, C_DIM, 1, cv2.LINE_AA)
        py += 16
        now = time.time()
        if now < self._gaze_lock_until:
            dot_col, gaze_txt = C_LOCK,  f'LOCKED  tag {self._gaze_locked_id}'
        elif self._gaze_tracking_id >= 0:
            dot_col, gaze_txt = C_TRACK, f'Tracking  tag {self._gaze_tracking_id}...'
        else:
            dot_col, gaze_txt = C_DIM,   'No tag in view'
        cv2.circle(canvas, (px + 5, py - 4), 5, dot_col, -1)
        cv2.putText(canvas, gaze_txt, (px + 15, py), FONT, 0.48, dot_col, 1, cv2.LINE_AA)
        py += 14

        # Separator
        cv2.line(canvas, (px, py), (WIN_W - 8, py), BORDER, 1)
        py += 10

        # Log
        cv2.putText(canvas, 'LOG', (px, py), FONT, 0.36, C_DIM, 1, cv2.LINE_AA)
        py += 16
        for i, entry in enumerate(self._log):
            alpha = max(0.4, 1.0 - i * 0.12)
            col = tuple(int(c * alpha) for c in C_WHITE)
            # truncate if too long
            max_chars = 34
            display = entry if len(entry) <= max_chars else entry[:max_chars] + '…'
            cv2.putText(canvas, display, (px, py), FONT, 0.38, col, 1, cv2.LINE_AA)
            py += 17

        cv2.imshow('S26 · Toyota Innovation Challenge', canvas)

        key = cv2.waitKey(1) & 0xFF
        if key == KEY_Q:
            cv2.destroyAllWindows()
            rclpy.shutdown()
        elif key == KEY_TAB:
            self._show_gaze = not self._show_gaze
        elif key == KEY_H:
            self._flip_h = not self._flip_h
        elif key == KEY_V:
            self._flip_v = not self._flip_v

    def _draw_gaze_overlay(self, cam):
        h, w = cam.shape[:2]
        now = time.time()
        if now < self._gaze_lock_until:
            text, colour, scale, thick = f'LOCKED  tag {self._gaze_locked_id}', C_LOCK, 0.9, 2
        elif self._gaze_tracking_id >= 0:
            text, colour, scale, thick = f'Tracking  tag {self._gaze_tracking_id}...', C_TRACK, 0.65, 1
        else:
            return
        tw = cv2.getTextSize(text, FONT, scale, thick)[0][0]
        tx, ty = (w - tw) // 2, h - 18
        overlay = cam.copy()
        cv2.rectangle(overlay, (0, ty - 22), (w, ty + 8), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, cam, 0.5, 0, cam)
        cv2.putText(cam, text, (tx, ty), FONT, scale, colour, thick, cv2.LINE_AA)

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
