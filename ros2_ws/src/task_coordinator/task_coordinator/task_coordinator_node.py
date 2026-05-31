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

    def __init__(self):
        super().__init__('task_coordinator_node')

        self.declare_parameter('min_reach_mm', 135.0)
        self.declare_parameter('max_reach_mm', 320.0)
        self.declare_parameter('z_safe', 40.0)
        self.declare_parameter('z_pick', -45.0)
        self.declare_parameter('z_drop', 10.0)

        self._state = State.IDLE
        self._latest_tags: TagDetectionArray | None = None
        self._latest_arm: Point | None = None

        self.create_subscription(Int32, '/gaze/gazed_tag_id', self._cb_gaze_tag, 10)
        self.create_subscription(TagDetectionArray, '/workspace/tag_detections', self._cb_tags, 10)
        self.create_subscription(Point, '/workspace/arm_position', self._cb_arm, 10)

        self._state_pub  = self.create_publisher(String, '~/state',  10)
        self._status_pub = self.create_publisher(String, '~/status', 10)
        self.create_timer(1.0, self._publish_state)

        self._move_xyz_cli = self.create_client(MoveToXYZ,      '/dobot_arm_node/move_to_xyz')
        self._gripper_cli  = self.create_client(GripperControl, '/dobot_arm_node/gripper_control')
        self._home_cli     = self.create_client(Trigger,        '/dobot_arm_node/home')

        self._status('Waiting for arm...')
        self.create_timer(0.5, self._startup_check)

    def _startup_check(self):
        if hasattr(self, '_startup_done'):
            return
        if not all(cli.wait_for_service(timeout_sec=0.1) for cli in [self._gripper_cli, self._home_cli]):
            self._status('Arm not ready yet — retrying...')
            return
        self._startup_done = True
        self._status('Closing gripper')

        def _close_done(_):
            self._status('Homing')
            self._home_cli.call_async(Trigger.Request()).add_done_callback(
                lambda _: self._status('Arm ready — waiting for gaze lock')
            )

        req = GripperControl.Request()
        req.open = False
        self._gripper_cli.call_async(req).add_done_callback(_close_done)

    def _status(self, text: str):
        msg = String(); msg.data = text
        self._status_pub.publish(msg)
        self.get_logger().info(text)

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

        if self._latest_tags is None:
            self.get_logger().warn(f'Tag {tag_id} locked — no overhead detections yet')
            return

        match = next((d for d in self._latest_tags.detections if d.tag_id == tag_id), None)
        if match is None:
            self.get_logger().warn(f'Tag {tag_id} not visible in overhead camera')
            return

        if not self._in_reach(match.robot_x, match.robot_y, min_r, max_r):
            self.get_logger().warn(f'Tag {tag_id} out of reach ({match.robot_x:.0f}, {match.robot_y:.0f}) mm')
            return

        if self._latest_arm is None:
            self.get_logger().warn('No hand detected')
            return

        if not self._in_reach(self._latest_arm.x, self._latest_arm.y, min_r, max_r):
            self.get_logger().warn(f'Hand out of reach ({self._latest_arm.x:.0f}, {self._latest_arm.y:.0f}) mm')
            return

        pick_x, pick_y = match.robot_x, match.robot_y
        drop_x, drop_y = self._latest_arm.x, self._latest_arm.y

        self._status(f'Executing: pick ({pick_x:.0f}, {pick_y:.0f}) → drop ({drop_x:.0f}, {drop_y:.0f})')
        self._execute_pick_and_place(pick_x, pick_y, drop_x, drop_y)

    @staticmethod
    def _in_reach(x, y, min_r, max_r):
        return min_r <= math.sqrt(x**2 + y**2) <= max_r

    def _sync_call(self, cli, req, timeout=15.0):
        event = threading.Event()
        cli.call_async(req).add_done_callback(lambda _: event.set())
        event.wait(timeout=timeout)

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

                self._status('Opening gripper');    gripper(True)
                self._status('Moving to pick');     move(pick_x, pick_y, z_safe)
                self._status('Descending');          move(pick_x, pick_y, z_pick)
                self._status('Gripping');            gripper(False)
                self._status('Moving to drop');     move(drop_x, drop_y, z_safe)
                self._status('Descending to hand'); move(drop_x, drop_y, z_drop)
                self._status('Releasing');           gripper(True)
                self._status('Lifting away');        move(drop_x, drop_y, z_safe)
                self._status('Closing gripper');     gripper(False)
                self._status('Homing');              self._sync_call(self._home_cli, Trigger.Request())
            finally:
                self._state = State.IDLE
                self._status('Ready — waiting for next gaze lock')

        threading.Thread(target=run, daemon=True).start()

    def _publish_state(self):
        msg = String(); msg.data = self._state.name
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
