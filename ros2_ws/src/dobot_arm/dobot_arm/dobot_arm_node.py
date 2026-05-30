import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from pick_interfaces.msg import RobotPose
from pick_interfaces.srv import GripperControl, MoveJoints, MoveToXYZ

from dobot_arm import dobot_hardware


class DobotArmNode(Node):
    def __init__(self):
        super().__init__('dobot_arm_node')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('rotate_angle', 0.0)

        port = self.get_parameter('serial_port').value
        self.get_logger().info(f'Connecting to Dobot on {port} …')
        self.dobot = dobot_hardware.connect(port)
        dobot_hardware.initialize_robot(self.dobot)

        # --- publishers ---
        self._pose_pub = self.create_publisher(RobotPose, '~/pose', 10)
        self._pose_timer = self.create_timer(0.1, self._publish_pose)  # 10 Hz

        # --- services ---
        self.create_service(MoveToXYZ, '~/move_to_xyz', self._cb_move_to_xyz)
        self.create_service(MoveJoints, '~/move_joints', self._cb_move_joints)
        self.create_service(GripperControl, '~/gripper_control', self._cb_gripper_control)
        self.create_service(Trigger, '~/home', self._cb_home)
        self.create_service(Trigger, '~/rotate_end_effector', self._cb_rotate_end_effector)

        self.get_logger().info('dobot_arm_node ready')

    # -----------------------------------------------------------------------
    # Publisher callback
    # -----------------------------------------------------------------------

    def _publish_pose(self):
        msg = RobotPose()
        try:
            x, y, z, r, j1, j2, j3, j4 = dobot_hardware.get_pose(self.dobot)
            msg.x, msg.y, msg.z, msg.r = x, y, z, r
            msg.j1, msg.j2, msg.j3, msg.j4 = j1, j2, j3, j4
        except Exception as e:
            self.get_logger().warn(f'pose read failed: {e}')
        self._pose_pub.publish(msg)

    # -----------------------------------------------------------------------
    # Service callbacks
    # -----------------------------------------------------------------------

    def _cb_move_to_xyz(self, request, response):
        self.get_logger().info(
            f'move_to_xyz: x={request.x} y={request.y} z={request.z} r={request.r_head}'
        )
        try:
            dobot_hardware.move_to_xyz(self.dobot, request.x, request.y, request.z, request.r_head)
            response.success = True
            response.message = 'ok'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _cb_move_joints(self, request, response):
        self.get_logger().info(
            f'move_joints: j1={request.j1} j2={request.j2} j3={request.j3} j4={request.j4}'
        )
        try:
            dobot_hardware.move_joint_angles(self.dobot, request.j1, request.j2, request.j3, request.j4)
            response.success = True
            response.message = 'ok'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _cb_gripper_control(self, request, response):
        self.get_logger().info(f'gripper_control: open={request.open}')
        try:
            if request.open:
                dobot_hardware.open_gripper(self.dobot)
            else:
                dobot_hardware.close_gripper(self.dobot)
            response.success = True
        except Exception as e:
            self.get_logger().error(str(e))
            response.success = False
        return response

    def _cb_home(self, request, response):
        self.get_logger().info('homing robot')
        try:
            dobot_hardware.move_to_home(self.dobot)
            response.success = True
            response.message = 'homed'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _cb_rotate_end_effector(self, request, response):
        angle = self.get_parameter('rotate_angle').value
        self.get_logger().info(f'rotate_end_effector: angle={angle}')
        try:
            dobot_hardware.rotate_end_effector(self.dobot, angle)
            response.success = True
            response.message = f'rotated to {angle}°'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DobotArmNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
