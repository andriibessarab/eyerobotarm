from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from pick_interfaces.msg import ObjectDetection
from pick_interfaces.srv import GripperControl, MoveToXYZ


class State(Enum):
    IDLE = auto()
    DETECTING_TARGET = auto()
    EXECUTING = auto()


class TaskCoordinatorNode(Node):
    """
    Orchestrates the pick-and-place loop:
      IDLE → DETECTING_TARGET → EXECUTING → IDLE

    Subscribes to object detections, calls Dobot arm services to pick and
    deliver the object to the user.
    """

    def __init__(self):
        super().__init__('task_coordinator_node')

        self._state = State.IDLE
        self._pending_target: ObjectDetection | None = None

        # --- subscribers ---
        self._det_sub = self.create_subscription(
            ObjectDetection,
            '/object_detection/detected_objects',
            self._cb_detection,
            10,
        )

        # --- publishers ---
        self._state_pub = self.create_publisher(String, '~/state', 10)
        self._state_timer = self.create_timer(1.0, self._publish_state)

        # --- service clients ---
        self._move_xyz_cli = self.create_client(MoveToXYZ, '/dobot_arm/move_to_xyz')
        self._gripper_cli = self.create_client(GripperControl, '/dobot_arm/gripper_control')
        self._home_cli = self.create_client(Trigger, '/dobot_arm/home')

        self.get_logger().info('task_coordinator_node ready — waiting for detections')

    # -----------------------------------------------------------------------
    # Detection callback
    # -----------------------------------------------------------------------

    def _cb_detection(self, msg: ObjectDetection):
        if self._state != State.IDLE:
            return

        self._pending_target = msg
        self._state = State.DETECTING_TARGET
        self.get_logger().info(
            f'target locked: label={msg.label} robot=({msg.robot_x:.1f}, {msg.robot_y:.1f})'
        )

        # TODO: add gaze-locking stability check before committing to EXECUTING
        #       e.g. require N consecutive frames on the same object before proceeding

        self._execute_pick_and_place(msg)

    # -----------------------------------------------------------------------
    # Pick-and-place execution
    # -----------------------------------------------------------------------

    def _execute_pick_and_place(self, target: ObjectDetection):
        self._state = State.EXECUTING
        self.get_logger().info('executing pick-and-place')

        # TODO: implement the full async pick-and-place sequence using
        #       call_async() for each service step:
        #
        #   1. open gripper
        #   2. move_to_xyz(target.robot_x, target.robot_y, Z_SAFE)
        #   3. move_to_xyz(target.robot_x, target.robot_y, Z_PICK)
        #   4. close gripper
        #   5. move_to_xyz(target.robot_x, target.robot_y, Z_SAFE)
        #   6. move to user handoff position
        #   7. open gripper
        #   8. home
        #   Then: self._state = State.IDLE

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
