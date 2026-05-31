import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from pupil_apriltags import Detector
import cv2

from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray, Point


class AprilTagWorkspaceNode(Node):
    """
    Subscribes to the overhead workspace camera, detects all AprilTags in each
    frame, and publishes an AprilTagDetectionArray with only tag id and center
    position populated.
    """

    def __init__(self):
        super().__init__('apriltag_workspace_node')

        self._bridge = CvBridge()
        self.declare_parameter('april_tag_family', 'tag16h5')
        self.declare_parameter('min_decision_margin', 20.0)
        self._april_tag_family = self.get_parameter('april_tag_family').value
        self._min_decision_margin = float(self.get_parameter('min_decision_margin').value)
        self.detector = Detector(families=self._april_tag_family)
        self.get_logger().info(
            f'AprilTag detector family: {self._april_tag_family}, '
            f'min decision margin: {self._min_decision_margin:.1f}'
        )

        self._sub = self.create_subscription(
            Image, 'workspace_camera/image_raw', self._cb_frame, 10
        )
        self._pub = self.create_publisher(AprilTagDetectionArray, 'workspace/tag_detections', 10)

        self.get_logger().info('apriltag_workspace_node ready')

    def _cb_frame(self, msg: Image):
        gray = cv2.cvtColor(self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'), cv2.COLOR_BGR2GRAY)
        tags = self.detector.detect(gray)
        self.get_logger().info(
            f'AprilTag detections in frame: {len(tags)}',
            throttle_duration_sec=2.0,
        )
        detections = []

        for tag in tags:
            if float(tag.decision_margin) < self._min_decision_margin:
                continue

            det = AprilTagDetection()
            det.id = int(tag.tag_id)
            det.decision_margin = float(tag.decision_margin)

            centre = Point()
            centre.x = float(tag.center[0])
            centre.y = float(tag.center[1])
            det.centre = centre
            detections.append(det)

        array_msg = AprilTagDetectionArray()
        array_msg.detections = detections
        self._pub.publish(array_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagWorkspaceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
