from enum import Enum, auto

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger

from pick_interfaces.msg import TagDetectionArray
from pick_interfaces.srv import GripperControl, MoveToXYZ


Z_SAFE = 40.0   # mm — clearance height for horizontal moves
Z_PICK = -25.0  # mm — height to grip object


class State(Enum):
    IDLE = auto()
    EXECUTING = auto()


class TaskCoordinatorNode(Node):
    """
    Orchestrates pick-and-place:
      IDLE → (gaze lock received) → EXECUTING → IDLE

    Gaze lock confirmation is handled upstream by apriltag_gaze_node.
    This node trusts whatever tag_id arrives on /gaze/gazed_tag_id as
    already confirmed — no additional stability check here.

    Pick position:  looked up by tag_id in /workspace/tag_detections
    Drop position:  latest /workspace/arm_position (falls back to param)
    """

    def __init__(self):
        super().__init__('task_coordinator_node')

        self.declare_parameter('drop_fallback_x', 200.0)
        self.declare_parameter('drop_fallback_y', 0.0)

        self._state = State.IDLE

        # latest data from upstream nodes
        self._latest_tags: TagDetectionArray | None = None
        self._latest_arm: Point | None = None

        # --- subscribers ---
        self.create_subscription(Int32, '/gaze/gazed_tag_id', self._cb_gaze_tag, 10)
        self.create_subscription(TagDetectionArray, '/workspace/tag_detections', self._cb_tags, 10)
        self.create_subscription(Point, '/workspace/arm_position', self._cb_arm, 10)

        # --- publishers ---
        self._state_pub = self.create_publisher(String, '~/state', 10)
        self.create_timer(1.0, self._publish_state)

        # --- service clients ---
        self._move_xyz_cli = self.create_client(MoveToXYZ, '/dobot_arm_node/move_to_xyz')
        self._gripper_cli  = self.create_client(GripperControl, '/dobot_arm_node/gripper_control')
        self._home_cli     = self.create_client(Trigger, '/dobot_arm_node/home')

        self.get_logger().info('task_coordinator_node ready — waiting for gaze lock')

    # -----------------------------------------------------------------------
    # Incoming data callbacks
    # -----------------------------------------------------------------------

    def _cb_tags(self, msg: TagDetectionArray):
        self._latest_tags = msg

    def _cb_arm(self, msg: Point):
        self._latest_arm = msg

    def _cb_gaze_tag(self, msg: Int32):
        tag_id = msg.data
        if tag_id < 0 or self._state != State.IDLE:
            return

        # Look up this tag's robot position in the latest overhead detection
        if self._latest_tags is None:
            self.get_logger().warn(f'Gaze locked tag {tag_id} but no tag detections available yet')
            return

        match = next((d for d in self._latest_tags.detections if d.tag_id == tag_id), None)
        if match is None:
            self.get_logger().warn(f'Gaze locked tag {tag_id} not visible in overhead camera')
            return

        pick_x, pick_y = match.robot_x, match.robot_y

        if self._latest_arm is not None:
            drop_x, drop_y = self._latest_arm.x, self._latest_arm.y
        else:
            drop_x = self.get_parameter('drop_fallback_x').value
            drop_y = self.get_parameter('drop_fallback_y').value
            self.get_logger().warn('No arm position detected — using fallback drop position')

        self.get_logger().info(
            f'Executing: pick tag={tag_id} at ({pick_x:.0f}, {pick_y:.0f}) '
            f'→ drop at ({drop_x:.0f}, {drop_y:.0f})'
        )
        self._execute_pick_and_place(pick_x, pick_y, drop_x, drop_y)

    # -----------------------------------------------------------------------
    # Pick-and-place execution — chained async service calls
    # -----------------------------------------------------------------------

    def _execute_pick_and_place(self, pick_x, pick_y, drop_x, drop_y):
        self._state = State.EXECUTING

        def _gripper(open_: bool, next_cb):
            req = GripperControl.Request()
            req.open = open_
            self._gripper_cli.call_async(req).add_done_callback(lambda _: next_cb())

        def _move(x, y, z, next_cb):
            req = MoveToXYZ.Request()
            req.x, req.y, req.z, req.r_head = x, y, z, 0.0
            self._move_xyz_cli.call_async(req).add_done_callback(lambda _: next_cb())

        def _home(next_cb):
            self._home_cli.call_async(Trigger.Request()).add_done_callback(lambda _: next_cb())

        def _done():
            self._state = State.IDLE
            self.get_logger().info('Pick-and-place complete — IDLE')

        # Chain: open → move safe → move pick → close → lift safe → move drop → open → home → done
        _gripper(True,  lambda: _move(pick_x, pick_y, Z_SAFE,
        lambda: _move(pick_x, pick_y, Z_PICK,
        lambda: _gripper(False, lambda: _move(pick_x, pick_y, Z_SAFE,
        lambda: _move(drop_x, drop_y, Z_SAFE,
        lambda: _gripper(True,  lambda: _home(_done))))))))

    # -----------------------------------------------------------------------
    # State publisher
    # -----------------------------------------------------------------------

    def _publish_state(self):
        msg = String()
        msg.data = self._state.name
        self._state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TaskCoordinatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
