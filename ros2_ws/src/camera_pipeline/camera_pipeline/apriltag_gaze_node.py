import math
from datetime import datetime

import cv2
import rclpy
from pyapriltags import Detector
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class AprilTagGazeNode(Node):
    """
    Subscribes to the gaze camera, detects tag36h11 AprilTags in each frame,
    and publishes the tag ID that has been stably centered for `stare_time`
    seconds. Publishes -1 when no tag is locked.

    Logic ported from glasses_detection/apriltags_detection.py.
    """

    def __init__(self):
        super().__init__('apriltag_gaze_node')

        self.declare_parameter('stare_time', 1.0)
        self.declare_parameter('center_tolerance', 50)

        self._bridge = CvBridge()

        self._detector = Detector(
            families='tag36h11',
            nthreads=1,
            quad_decimate=2.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0,
        )

        # [tag_id, x_distance, start_time] while accumulating; None when idle
        self._candidate = None

        self._sub = self.create_subscription(
            Image, '/gaze_camera/image_raw', self._cb_frame, 10
        )
        self._pub          = self.create_publisher(Int32, '/gaze/gazed_tag_id', 10)
        self._tracking_pub = self.create_publisher(Int32, '~/tracking',         10)

        self.get_logger().info('apriltag_gaze_node ready')

    def _publish(self, tag_id: int):
        msg = Int32()
        msg.data = tag_id
        self._pub.publish(msg)

    def _publish_tracking(self):
        msg = Int32()
        msg.data = self._candidate[0] if self._candidate is not None else -1
        self._tracking_pub.publish(msg)

    def _cb_frame(self, msg: Image):
        stare_time       = self.get_parameter('stare_time').value
        center_tolerance = self.get_parameter('center_tolerance').value

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = cv2.flip(frame, -1)
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detections = self._detector.detect(gray)

        # Drop candidate if its tag is no longer visible
        visible_ids = [d.tag_id for d in detections]
        if self._candidate is not None and self._candidate[0] not in visible_ids:
            self._candidate = None

        frame_x_center = gray.shape[1] / 2

        best_tag_id  = None
        best_x_dist  = float('inf')

        for detection in detections:
            if detection.decision_margin < 10:
                continue

            x_dist = math.fabs(detection.center[0] - frame_x_center)
            if x_dist > center_tolerance:
                continue

            if x_dist < best_x_dist:
                best_x_dist = x_dist
                best_tag_id = detection.tag_id

        if best_tag_id is None:
            self._candidate = None
            self._publish(-1)
            self._publish_tracking()
            return

        if self._candidate is None or self._candidate[0] != best_tag_id:
            self._candidate = [best_tag_id, best_x_dist, datetime.now()]
            self._publish(-1)
            self._publish_tracking()
            return

        elapsed = (datetime.now() - self._candidate[2]).total_seconds()
        if elapsed < stare_time:
            self._publish(-1)
            self._publish_tracking()
            return

        # Stare time exceeded — lock confirmed
        self.get_logger().info(
            f'Gaze locked on tag {best_tag_id} after {elapsed:.1f}s'
        )
        self._publish(best_tag_id)
        self._candidate = None
        self._publish_tracking()


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
