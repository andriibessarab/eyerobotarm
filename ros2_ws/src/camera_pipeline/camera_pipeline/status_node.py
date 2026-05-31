import time
from collections import deque
from datetime import datetime

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String

from pick_interfaces.msg import TagDetectionArray
from geometry_msgs.msg import Point

WIN_W, WIN_H = 640, 520
MAX_LOG      = 7
FONT         = cv2.FONT_HERSHEY_SIMPLEX

# colours (BGR)
C_BG      = (22,  22,  22)
C_PANEL   = (32,  32,  32)
C_BORDER  = (55,  55,  55)
C_WHITE   = (220, 220, 220)
C_DIM     = ( 90,  90,  90)
C_IDLE    = ( 80, 200,  80)
C_EXEC    = ( 60, 200, 240)
C_WARN    = ( 60, 140, 255)
C_OK      = ( 60, 200,  80)
C_FAIL    = ( 60,  60, 200)
C_GAZE    = (100, 180, 255)

STALE_SEC = 2.0   # seconds before a detection is considered stale


def _ascii(text: str) -> str:
    """Replace common unicode symbols so cv2.putText doesn't show ???."""
    return (text
            .replace('→', '->')   # →
            .replace('°', 'deg')  # °
            .replace('·', '.')    # ·
            .replace('—', '--')   # —
            .replace('–', '-')    # –
            .encode('ascii', 'replace').decode('ascii'))


def _dot(frame, x, y, on, r=7):
    col = C_OK if on else C_FAIL
    cv2.circle(frame, (x, y), r, col, -1)
    cv2.circle(frame, (x, y), r, tuple(c // 2 for c in col), 1)


class StatusNode(Node):

    def __init__(self):
        super().__init__('status_node')

        self._coord_state  = 'WAITING'
        self._latest_action = 'Waiting for arm...'
        self._log: deque[str] = deque(maxlen=MAX_LOG)

        # Detection state + timestamps
        self._ws_tag_time   = 0.0   # last time workspace tag was seen
        self._ws_tag_id     = -1
        self._hand_time     = 0.0   # last time hand position was received
        self._palm_up       = False
        self._palm_time     = 0.0
        self._gaze_tracking = -1    # tag being accumulated
        self._gaze_locked   = -1
        self._gaze_lock_t   = 0.0

        self.create_subscription(String,          '/coordinator/state',            self._cb_state,    10)
        self.create_subscription(String,          '/coordinator/status',           self._cb_status,   10)
        self.create_subscription(TagDetectionArray, '/workspace/tag_detections',   self._cb_ws_tags,  10)
        self.create_subscription(Point,           '/workspace/arm_position',       self._cb_hand,     10)
        self.create_subscription(Bool,            '/workspace/palm',               self._cb_palm,     10)
        self.create_subscription(Int32,           '/apriltag_gaze/tracking',       self._cb_tracking, 10)
        self.create_subscription(Int32,           '/gaze/gazed_tag_id',            self._cb_locked,   10)

        self.create_timer(1.0 / 30.0, self._draw)
        self.get_logger().info('status_node ready')

    def _cb_state(self, msg):
        self._coord_state = msg.data

    def _cb_status(self, msg):
        self._latest_action = msg.data
        ts = datetime.now().strftime('%H:%M:%S')
        self._log.appendleft(f'{ts}  {_ascii(msg.data)}')

    def _cb_ws_tags(self, msg):
        if msg.detections:
            self._ws_tag_time = time.time()
            self._ws_tag_id   = msg.detections[0].tag_id

    def _cb_hand(self, msg):
        self._hand_time = time.time()

    def _cb_palm(self, msg):
        self._palm_up   = msg.data
        self._palm_time = time.time()

    def _cb_tracking(self, msg):
        self._gaze_tracking = msg.data

    def _cb_locked(self, msg):
        if msg.data >= 0:
            self._gaze_locked = msg.data
            self._gaze_lock_t = time.time()

    def _draw(self):
        now    = time.time()
        canvas = np.full((WIN_H, WIN_W, 3), C_BG, dtype=np.uint8)

        # ── Header ───────────────────────────────────────────────────────
        cv2.rectangle(canvas, (0, 0), (WIN_W, 32), C_PANEL, -1)
        cv2.line(canvas, (0, 32), (WIN_W, 32), C_BORDER, 1)
        cv2.putText(canvas, 'S26  TOYOTA INNOVATION CHALLENGE', (12, 22),
                    FONT, 0.5, C_DIM, 1, cv2.LINE_AA)

        # ── State badge ───────────────────────────────────────────────────
        py = 44
        state_col = C_IDLE if self._coord_state == 'IDLE' else (
                    C_EXEC if self._coord_state == 'EXECUTING' else C_WARN)
        cv2.rectangle(canvas, (12, py), (12 + 160, py + 30),
                      tuple(c // 6 for c in state_col), -1)
        cv2.rectangle(canvas, (12, py), (12 + 160, py + 30), state_col, 1)
        cv2.putText(canvas, self._coord_state, (20, py + 21),
                    FONT, 0.7, state_col, 1, cv2.LINE_AA)

        # Gaze lock badge (top-right)
        if now - self._gaze_lock_t < 3.0 and self._gaze_locked >= 0:
            gl_txt = f'LOCKED  tag {self._gaze_locked}'
            cv2.putText(canvas, gl_txt, (WIN_W - 185, py + 21),
                        FONT, 0.6, C_OK, 1, cv2.LINE_AA)
        elif self._gaze_tracking >= 0:
            gt_txt = f'Tracking  tag {self._gaze_tracking}...'
            cv2.putText(canvas, gt_txt, (WIN_W - 200, py + 21),
                        FONT, 0.5, C_GAZE, 1, cv2.LINE_AA)

        py += 42

        # ── Divider ───────────────────────────────────────────────────────
        cv2.line(canvas, (0, py), (WIN_W, py), C_BORDER, 1)
        py += 10

        # ── Detection status ──────────────────────────────────────────────
        cv2.putText(canvas, 'DETECTION STATUS', (12, py),
                    FONT, 0.38, C_DIM, 1, cv2.LINE_AA)
        py += 18

        ws_tag_ok   = (now - self._ws_tag_time) < STALE_SEC
        hand_ok     = (now - self._hand_time)   < STALE_SEC
        palm_ok     = (now - self._palm_time)   < STALE_SEC and self._palm_up
        gaze_ok     = self._gaze_tracking >= 0 or (now - self._gaze_lock_t < 3.0)

        checks = [
            (ws_tag_ok,   f'Workspace tag' + (f' #{self._ws_tag_id}' if ws_tag_ok and self._ws_tag_id >= 0 else '')),
            (hand_ok,     'Hand in workspace'),
            (palm_ok,     'Palm facing up'),
            (gaze_ok,     'Gaze on tag'),
        ]

        col_w = WIN_W // 2
        for i, (ok, label) in enumerate(checks):
            cx = 20  + (i % 2) * col_w
            cy = py  + (i // 2) * 28
            _dot(canvas, cx, cy, ok)
            cv2.putText(canvas, label, (cx + 16, cy + 5),
                        FONT, 0.48, C_WHITE if ok else C_DIM, 1, cv2.LINE_AA)

        py += 28 * 2 + 8

        # ── Divider ───────────────────────────────────────────────────────
        cv2.line(canvas, (0, py), (WIN_W, py), C_BORDER, 1)
        py += 10

        # ── Current action ────────────────────────────────────────────────
        cv2.putText(canvas, 'CURRENT ACTION', (12, py),
                    FONT, 0.38, C_DIM, 1, cv2.LINE_AA)
        py += 18

        words = _ascii(self._latest_action).split()
        lines, cur = [], ''
        for w in words:
            candidate = (cur + ' ' + w).strip()
            if cv2.getTextSize(candidate, FONT, 0.58, 1)[0][0] < WIN_W - 24:
                cur = candidate
            else:
                lines.append(cur); cur = w
        if cur:
            lines.append(cur)
        for line in lines[:2]:
            cv2.putText(canvas, line, (12, py), FONT, 0.58, C_WHITE, 1, cv2.LINE_AA)
            py += 22
        py += 4

        # ── Divider ───────────────────────────────────────────────────────
        cv2.line(canvas, (0, py), (WIN_W, py), C_BORDER, 1)
        py += 10

        # ── Log ───────────────────────────────────────────────────────────
        cv2.putText(canvas, 'LOG', (12, py), FONT, 0.38, C_DIM, 1, cv2.LINE_AA)
        py += 16
        for i, entry in enumerate(self._log):
            alpha = max(0.3, 1.0 - i * 0.1)
            col = tuple(int(c * alpha) for c in C_WHITE)
            display = entry if len(entry) <= 58 else entry[:57] + '>'
            cv2.putText(canvas, display, (12, py), FONT, 0.4, col, 1, cv2.LINE_AA)
            py += 18

        cv2.imshow('S26 Status', canvas)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StatusNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
