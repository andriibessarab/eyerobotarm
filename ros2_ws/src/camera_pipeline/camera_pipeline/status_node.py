from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String


WIN_W, WIN_H = 600, 400
MAX_LOG = 8  # status message history lines

# colours (BGR)
C_BG       = (30,  30,  30)
C_IDLE     = (80, 200,  80)
C_EXEC     = (60, 200, 240)
C_TEXT     = (220, 220, 220)
C_DIM      = (120, 120, 120)
C_GAZE     = (100, 180, 255)
C_DIVIDER  = (70,  70,  70)

FONT       = cv2.FONT_HERSHEY_SIMPLEX


class StatusNode(Node):
    """Displays a live OpenCV window showing task coordinator state and step-by-step status."""

    def __init__(self):
        super().__init__('status_node')

        self._coord_state = 'IDLE'
        self._gaze_id = -1
        self._log: deque[str] = deque(maxlen=MAX_LOG)
        self._latest_status = ''

        self.create_subscription(String, '/task_coordinator_node/state',  self._cb_state,  10)
        self.create_subscription(String, '/task_coordinator_node/status', self._cb_status, 10)
        self.create_subscription(Int32,  '/gaze/gazed_tag_id',            self._cb_gaze,   10)

        self.create_timer(1.0 / 30.0, self._draw)
        self.get_logger().info('status_node ready — press Q in window to quit')

    def _cb_state(self, msg: String):
        self._coord_state = msg.data

    def _cb_status(self, msg: String):
        self._latest_status = msg.data
        self._log.appendleft(msg.data)

    def _cb_gaze(self, msg: Int32):
        self._gaze_id = msg.data

    def _draw(self):
        frame = np.full((WIN_H, WIN_W, 3), C_BG, dtype=np.uint8)

        # ── header bar ──────────────────────────────────────────────────
        state_colour = C_IDLE if self._coord_state == 'IDLE' else C_EXEC
        cv2.rectangle(frame, (0, 0), (WIN_W, 60), (20, 20, 20), -1)
        cv2.putText(frame, 'ROBOT STATUS', (16, 22),
                    FONT, 0.5, C_DIM, 1, cv2.LINE_AA)
        cv2.putText(frame, self._coord_state, (16, 52),
                    FONT, 1.1, state_colour, 2, cv2.LINE_AA)

        # gaze tag indicator
        gaze_txt = f'Gaze: tag {self._gaze_id}' if self._gaze_id >= 0 else 'Gaze: —'
        cv2.putText(frame, gaze_txt, (WIN_W - 180, 52),
                    FONT, 0.55, C_GAZE, 1, cv2.LINE_AA)

        # ── divider ─────────────────────────────────────────────────────
        cv2.line(frame, (0, 64), (WIN_W, 64), C_DIVIDER, 1)

        # ── current action ──────────────────────────────────────────────
        cv2.putText(frame, 'Current action:', (16, 90),
                    FONT, 0.45, C_DIM, 1, cv2.LINE_AA)

        # word-wrap current status into two lines max
        words = self._latest_status.split()
        lines, cur = [], ''
        for w in words:
            candidate = (cur + ' ' + w).strip()
            if cv2.getTextSize(candidate, FONT, 0.62, 1)[0][0] < WIN_W - 32:
                cur = candidate
            else:
                lines.append(cur); cur = w
        if cur:
            lines.append(cur)

        for i, line in enumerate(lines[:2]):
            cv2.putText(frame, line, (16, 115 + i * 28),
                        FONT, 0.62, C_TEXT, 1, cv2.LINE_AA)

        # ── divider ─────────────────────────────────────────────────────
        cv2.line(frame, (0, 175), (WIN_W, 175), C_DIVIDER, 1)

        # ── log ─────────────────────────────────────────────────────────
        cv2.putText(frame, 'Recent:', (16, 198),
                    FONT, 0.45, C_DIM, 1, cv2.LINE_AA)

        for i, entry in enumerate(self._log):
            alpha = max(0.35, 1.0 - i * 0.12)
            colour = tuple(int(c * alpha) for c in C_TEXT)
            cv2.putText(frame, entry, (16, 222 + i * 22),
                        FONT, 0.45, colour, 1, cv2.LINE_AA)

        cv2.imshow('Robot Status', frame)
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
