from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_offboard',
            executable='offboard_control',
            name='offboard_control_node',
            output='screen'
        ),
        Node(
            package='px4_offboard',
            executable='aruco_marker_detector',
            name='aruco_marker_detector_node',
            output='screen'
        ),
    ])

