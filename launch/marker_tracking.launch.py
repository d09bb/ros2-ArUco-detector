from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    aruco_node = Node(
        package='aruco_detector',
        executable='aruco_node',
        name='aruco_detector',
        output='screen'
    )

    marker_controller = Node(
        package='aruco_detector',
        executable='marker_position_controller.py',
        name='marker_position_controller',
        output='screen'
    )

    return LaunchDescription([
        aruco_node,
        marker_controller
    ])
