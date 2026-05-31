from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(
            'PROVIDED_CODE_PATH',
            '/home/andriibessarab/Desktop/S26-Toyota-Innovation-Challenge/provided_code',
        ),
        Node(
            package='dobot_arm',
            executable='dobot_arm_node',
            output='screen',
            parameters=[{'serial_port': '/dev/ttyUSB2'}],
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
            parameters=[{'camera_source': 'tcp://10.12.194.1:5000'}],
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
            parameters=[{'stare_time': 1.5}],
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
        Node(
            package='camera_pipeline',
            executable='preview_node',
            output='screen',
        ),
    ])
