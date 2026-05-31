import math
from datetime import datetime

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class AprilTagGazeNode(Node):
    """
    Subscribes to the gaze camera, detects tag16h5 AprilTags in each frame,
    and publishes the tag ID that has been stably centered for `stare_time`
    seconds. Publishes -1 when no tag is locked.

    Detection uses cv2.aruco with DICT_APRILTAG_16h5 (equivalent to tag16h5).
    Logic ported from glasses_detection/apriltags_detection.py.
    """

    def __init__(self):
        super().__init__('apriltag_gaze_node')

        self.declare_parameter('stare_time', 3.0)
        self.declare_parameter('center_tolerance', 150)
        self.declare_parameter('center_offset_y', 50)

        self._bridge = CvBridge()

        _dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)
        _params = cv2.aruco.DetectorParameters_create()
        self._detector = (_dict, _params)

        # [tag_id, distance, start_time] while accumulating; None when idle
        self._candidate = None

        self._sub = self.create_subscription(
            Image, '/gaze_camera/image_raw', self._cb_frame, 10
        )
        self._pub = self.create_publisher(Int32, '/gaze/gazed_tag_id', 10)

        self.get_logger().info('apriltag_gaze_node ready')

    def _publish(self, tag_id: int):
        msg = Int32()
        msg.data = tag_id
        self._pub.publish(msg)

    def _cb_frame(self, msg: Image):
        stare_time = self.get_parameter('stare_time').value
        center_tolerance = self.get_parameter('center_tolerance').value
        center_offset_y = self.get_parameter('center_offset_y').value

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape

        corners, ids, _ = cv2.aruco.detectMarkers(gray, *self._detector)

        if ids is None or len(ids) == 0:
            self._candidate = None
            self._publish(-1)
            return

        flat_ids = ids.flatten().tolist()

        # Drop candidate if its tag is no longer visible
        if self._candidate is not None and self._candidate[0] not in flat_ids:
            self._candidate = None

        frame_center = (w / 2, h / 2)

        # Find closest tag to (adjusted) frame center within tolerance
        best_tag_id = None
        best_dist = float('inf')
        for i, tag_id in enumerate(flat_ids):
            center = corners[i][0].mean(axis=0).copy()
            center[1] -= center_offset_y
            dist = math.dist(center, frame_center)
            if dist > center_tolerance:
                continue
            if dist < best_dist:
                best_dist = dist
                best_tag_id = tag_id

        if best_tag_id is None:
            self._candidate = None
            self._publish(-1)
            return

        if self._candidate is None or self._candidate[0] != best_tag_id:
            self._candidate = [best_tag_id, best_dist, datetime.now()]
            self._publish(-1)
            return

        elapsed = (datetime.now() - self._candidate[2]).total_seconds()
        if elapsed < stare_time:
            self._publish(-1)
            return

        # Stare time exceeded — lock confirmed
        self.get_logger().info(
            f'Gaze locked on tag {best_tag_id} after {elapsed:.1f}s'
        )
        self._publish(best_tag_id)
        self._candidate = None


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagGazeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
