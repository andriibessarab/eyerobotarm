import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class AprilTagGazeNode(Node):
    """
    Subscribes to the gaze camera, detects AprilTags in each frame, and publishes
    the tag ID that has been stably centered in the frame for `stability_frames`
    consecutive frames. Publishes -1 when no tag is locked.

    TODO: implement detection loop in _cb_frame
      1. cv2.aruco.detectMarkers(frame, DICT_APRILTAG_36H11)
      2. find marker closest to frame center within center_tolerance
      3. stability counter — publish tag_id when stable, else -1
    """

    def __init__(self):
        super().__init__('apriltag_gaze_node')

        self.declare_parameter('stability_frames', 60)
        self.declare_parameter('center_tolerance', 100)

        self._bridge = CvBridge()
        self._candidate_id = -1
        self._stable_count = 0

        self._sub = self.create_subscription(
            Image, '/gaze_camera/image_raw', self._cb_frame, 10
        )
        self._pub = self.create_publisher(Int32, '/gaze/gazed_tag_id', 10)

        self.get_logger().info('apriltag_gaze_node ready (TODO: detection not yet implemented)')

    def _cb_frame(self, msg: Image):
        # TODO: implement AprilTag detection and stability logic
        pass


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
