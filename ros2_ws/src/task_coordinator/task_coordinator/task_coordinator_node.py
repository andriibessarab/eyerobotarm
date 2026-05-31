import math
import threading
from enum import Enum, auto

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger

from pick_interfaces.msg import TagDetectionArray
from pick_interfaces.srv import GripperControl, MoveToXYZ


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
        self._move_xyz_cli = self.create_client(MoveToXYZ,      '/dobot_arm_node/move_to_xyz')
        self._gripper_cli  = self.create_client(GripperControl, '/dobot_arm_node/gripper_control')
        self._home_cli     = self.create_client(Trigger,        '/dobot_arm_node/home')

        self._status('Waiting for arm...')
        self.create_timer(0.5, self._startup_check)

    # -----------------------------------------------------------------------
    # Startup arm check
    # -----------------------------------------------------------------------

    def _startup_check(self):
        if hasattr(self, '_startup_done'):
            return
        services_ready = all(
            cli.wait_for_service(timeout_sec=0.1)
            for cli in [self._gripper_cli, self._home_cli]
        )
        if not services_ready:
            self._status('Arm not ready yet — retrying...')
            return
        self._startup_done = True

        self._status('Arm check: closing gripper')

        def _close_done(_):
            self._status('Arm check: homing')
            self._home_cli.call_async(Trigger.Request()).add_done_callback(_home_done)

        def _home_done(_):
            self._status('Arm ready — waiting for gaze lock')

        req = GripperControl.Request()
        req.open = False
        self._gripper_cli.call_async(req).add_done_callback(_close_done)

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

    def _sync_call(self, cli, req, timeout=15.0):
        event = threading.Event()
        result = [None]
        def cb(future):
            result[0] = future.result()
            event.set()
        cli.call_async(req).add_done_callback(cb)
        event.wait(timeout=timeout)
        return result[0]

    def _execute_pick_and_place(self, pick_x, pick_y, drop_x, drop_y):
        self._state = State.EXECUTING
        z_safe = self.get_parameter('z_safe').value
        z_pick = self.get_parameter('z_pick').value
        z_drop = self.get_parameter('z_drop').value

        def run():
            try:
                def gripper(open_):
                    req = GripperControl.Request(); req.open = open_
                    self._sync_call(self._gripper_cli, req)

                def move(x, y, z):
                    req = MoveToXYZ.Request()
                    req.x, req.y, req.z, req.r_head = x, y, z, 0.0
                    self._sync_call(self._move_xyz_cli, req)

                def home():
                    self._sync_call(self._home_cli, Trigger.Request())

                self._status('Opening gripper');  gripper(True)
                self._status('Moving to pick');   move(pick_x, pick_y, z_safe)
                self._status('Descending');        move(pick_x, pick_y, z_pick)
                self._status('Gripping');          gripper(False)
                self._status('Moving to drop');   move(drop_x, drop_y, z_safe)
                self._status('Descending to hand'); move(drop_x, drop_y, z_drop)
                self._status('Releasing');         gripper(True)
                self._status('Homing');            home()
            finally:
                self._state = State.IDLE
                self._status('Ready — waiting for next gaze lock')

        threading.Thread(target=run, daemon=True).start()

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
