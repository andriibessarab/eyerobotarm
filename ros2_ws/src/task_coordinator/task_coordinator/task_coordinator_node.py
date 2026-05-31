import math
from enum import Enum, auto

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger

from pick_interfaces.msg import TagDetectionArray
from pick_interfaces.srv import GripperControl, MoveToXYZ, SetSpeed


class State(Enum):
    IDLE = auto()
    EXECUTING = auto()


class TaskCoordinatorNode(Node):
    """
    Orchestrates gaze-guided pick-and-place.

    Flow:
      IDLE
        → gaze lock received (tag_id ≥ 0 from apriltag_gaze_node)
        → VALIDATE:
            • tag must be visible in overhead camera AND within Dobot reach
            • human hand must be visible in overhead camera AND within Dobot reach
        → EXECUTING (if both pass):
            1. open gripper
            2. move above pick position      (normal speed)
            3. descend to object             (normal speed)
            4. close gripper
            5. lift to safe height
            6. move above hand position      (normal speed)
            7. reduce to approach speed
            8. descend to hand               (slow — near human)
            9. open gripper
           10. restore normal speed → home → IDLE
        → IDLE (if validation fails — warn and wait for next gaze lock)
    """

    def __init__(self):
        super().__init__('task_coordinator_node')

        self.declare_parameter('min_reach_mm', 135.0)
        self.declare_parameter('max_reach_mm', 320.0)
        self.declare_parameter('z_safe', 40.0)
        self.declare_parameter('z_pick', -25.0)
        self.declare_parameter('z_drop', 0.0)
        self.declare_parameter('normal_velocity', 50)
        self.declare_parameter('approach_velocity', 20)
        self.declare_parameter('exec_timeout', 15.0)

        self._state = State.IDLE
        self._watchdog = None

        self._latest_tags: TagDetectionArray | None = None
        self._latest_arm: Point | None = None

        # --- subscribers ---
        self.create_subscription(Int32, '/gaze/gazed_tag_id', self._cb_gaze_tag, 10)
        self.create_subscription(TagDetectionArray, '/workspace/tag_detections', self._cb_tags, 10)
        self.create_subscription(Point, '/workspace/arm_position', self._cb_arm, 10)

        # --- publishers ---
        self._state_pub  = self.create_publisher(String, '~/state',  10)
        self._status_pub = self.create_publisher(String, '~/status', 10)
        self.create_timer(1.0, self._publish_state)

        # --- service clients ---
        self._move_xyz_cli  = self.create_client(MoveToXYZ,      '/dobot_arm_node/move_to_xyz')
        self._gripper_cli   = self.create_client(GripperControl, '/dobot_arm_node/gripper_control')
        self._home_cli      = self.create_client(Trigger,        '/dobot_arm_node/home')
        self._set_speed_cli = self.create_client(SetSpeed,       '/dobot_arm_node/set_speed')

        self._status('Waiting for arm...')
        self.create_timer(0.5, self._startup_check)

    # -----------------------------------------------------------------------
    # Startup arm check
    # -----------------------------------------------------------------------

    def _startup_check(self):
        self.destroy_timer(self._startup_timer if hasattr(self, '_startup_timer') else None)
        # Only run once
        if hasattr(self, '_startup_done'):
            return
        self._startup_done = True

        services_ready = all(
            cli.wait_for_service(timeout_sec=0.1)
            for cli in [self._gripper_cli, self._home_cli]
        )
        if not services_ready:
            self._status('Arm not ready yet — retrying...')
            self.create_timer(1.0, self._startup_check)
            return

        self._status('Arm check: open gripper')

        def _open_done(_):
            self._status('Arm check: close gripper')
            req = GripperControl.Request()
            req.open = False
            self._gripper_cli.call_async(req).add_done_callback(_close_done)

        def _close_done(_):
            self._status('Arm check: homing')
            self._home_cli.call_async(Trigger.Request()).add_done_callback(_home_done)

        def _home_done(_):
            self._status('Arm ready — waiting for gaze lock')

        req = GripperControl.Request()
        req.open = True
        self._gripper_cli.call_async(req).add_done_callback(_open_done)

    # -----------------------------------------------------------------------
    # Status helper
    # -----------------------------------------------------------------------

    def _status(self, text: str):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        self.get_logger().info(text)

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

        min_r = self.get_parameter('min_reach_mm').value
        max_r = self.get_parameter('max_reach_mm').value

        # --- validate tag ---
        if self._latest_tags is None:
            self.get_logger().warn(f'Gaze locked tag {tag_id} — no overhead detections yet')
            return

        match = next((d for d in self._latest_tags.detections if d.tag_id == tag_id), None)
        if match is None:
            self.get_logger().warn(f'Gaze locked tag {tag_id} not visible in overhead camera')
            return

        if not self._in_reach(match.robot_x, match.robot_y, min_r, max_r):
            self.get_logger().warn(
                f'Tag {tag_id} at ({match.robot_x:.0f}, {match.robot_y:.0f}) mm '
                f'is outside reachable range [{min_r:.0f}–{max_r:.0f}] mm'
            )
            return

        pick_x, pick_y = match.robot_x, match.robot_y

        # --- validate hand ---
        if self._latest_arm is None:
            self.get_logger().warn('No hand detected — cannot determine drop position')
            return

        if not self._in_reach(self._latest_arm.x, self._latest_arm.y, min_r, max_r):
            self.get_logger().warn(
                f'Hand at ({self._latest_arm.x:.0f}, {self._latest_arm.y:.0f}) mm '
                f'is outside reachable range [{min_r:.0f}–{max_r:.0f}] mm'
            )
            return

        drop_x, drop_y = self._latest_arm.x, self._latest_arm.y

        # --- check arm services are up ---
        for cli, name in [
            (self._move_xyz_cli, 'move_to_xyz'),
            (self._gripper_cli,  'gripper_control'),
            (self._home_cli,     'home'),
            (self._set_speed_cli,'set_speed'),
        ]:
            if not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f'Arm service {name} not available — is dobot_arm_node running?')
                self._status(f'ERROR: arm service {name} unavailable')
                return

        self._status(
            f'Gaze locked tag {tag_id} — pick ({pick_x:.0f}, {pick_y:.0f}) '
            f'→ hand ({drop_x:.0f}, {drop_y:.0f})'
        )
        self._execute_pick_and_place(pick_x, pick_y, drop_x, drop_y)

    # -----------------------------------------------------------------------
    # Reach check
    # -----------------------------------------------------------------------

    @staticmethod
    def _in_reach(robot_x: float, robot_y: float, min_r: float, max_r: float) -> bool:
        dist = math.sqrt(robot_x ** 2 + robot_y ** 2)
        return min_r <= dist <= max_r

    # -----------------------------------------------------------------------
    # Pick-and-place execution — chained async service calls
    # -----------------------------------------------------------------------

    def _execute_pick_and_place(self, pick_x, pick_y, drop_x, drop_y):
        self._state = State.EXECUTING
        timeout = self.get_parameter('exec_timeout').value
        self._watchdog = self.create_timer(timeout, self._on_timeout)

        z_safe     = self.get_parameter('z_safe').value
        z_pick     = self.get_parameter('z_pick').value
        z_drop     = self.get_parameter('z_drop').value
        normal_v   = self.get_parameter('normal_velocity').value
        approach_v = self.get_parameter('approach_velocity').value

        def _gripper(open_: bool, next_cb):
            req = GripperControl.Request()
            req.open = open_
            self._gripper_cli.call_async(req).add_done_callback(lambda _: next_cb())

        def _move(x, y, z, next_cb):
            req = MoveToXYZ.Request()
            req.x, req.y, req.z, req.r_head = x, y, z, 0.0
            self._move_xyz_cli.call_async(req).add_done_callback(lambda _: next_cb())

        def _speed(v, next_cb):
            req = SetSpeed.Request()
            req.velocity = v
            req.acceleration = v
            self._set_speed_cli.call_async(req).add_done_callback(lambda _: next_cb())

        def _home(next_cb):
            self._home_cli.call_async(Trigger.Request()).add_done_callback(lambda _: next_cb())

        def _done():
            if self._watchdog:
                self._watchdog.cancel()
                self._watchdog.destroy()
                self._watchdog = None
            self._state = State.IDLE
            self._status('Ready — waiting for next gaze lock')

        # Full sequence with status at each step
        self._status('Opening gripper')
        _gripper(True, lambda: (
            self._status('Moving above pick position'),
            _move(pick_x, pick_y, z_safe, lambda: (
                self._status('Descending to object'),
                _move(pick_x, pick_y, z_pick, lambda: (
                    self._status('Gripping object'),
                    _gripper(False, lambda: (
                        self._status('Object secured — lifting'),
                        _move(pick_x, pick_y, z_safe, lambda: (
                            self._status('Transiting to hand'),
                            _move(drop_x, drop_y, z_safe, lambda: (
                                self._status('Slowing for human approach'),
                                _speed(approach_v, lambda: (
                                    self._status('Descending to hand'),
                                    _move(drop_x, drop_y, z_drop, lambda: (
                                        self._status('Releasing object'),
                                        _gripper(True, lambda: (
                                            self._status('Delivery complete — returning home'),
                                            _speed(normal_v, lambda:
                                                _home(_done))
                                        ))
                                    ))
                                ))
                            ))
                        ))
                    ))
                ))
            ))
        ))

    # -----------------------------------------------------------------------
    # Watchdog
    # -----------------------------------------------------------------------

    def _on_timeout(self):
        self._status(
            f'ERROR: timed out after {self.get_parameter("exec_timeout").value:.0f}s — resetting'
        )
        if self._watchdog:
            self._watchdog.destroy()
            self._watchdog = None
        self._state = State.IDLE

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
