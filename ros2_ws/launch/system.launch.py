from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dobot_arm',
            executable='dobot_arm_node',
            name='dobot_arm',
            output='screen',
            parameters=[{'serial_port': '/dev/ttyUSB0'}],
        ),
        Node(
            package='camera_pipeline',
            executable='workspace_camera_node',
            name='workspace_camera',
            output='screen',
            parameters=[{'camera_source': '0'}],
        ),
        Node(
            package='camera_pipeline',
            executable='gaze_camera_node',
            name='gaze_camera',
            output='screen',
            parameters=[{'camera_source': '1'}],
        ),
        Node(
            package='camera_pipeline',
            executable='apriltag_workspace_node',
            name='apriltag_workspace',
            output='screen',
        ),
        Node(
            package='camera_pipeline',
            executable='apriltag_gaze_node',
            name='apriltag_gaze',
            output='screen',
        ),
        Node(
            package='camera_pipeline',
            executable='arm_detection_node',
            name='arm_detection',
            output='screen',
        ),
        Node(
            package='task_coordinator',
            executable='task_coordinator_node',
            name='coordinator',
            output='screen',
        ),
    ])
