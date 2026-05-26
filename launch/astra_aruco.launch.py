import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    astra_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('astra_camera'),
                'launch',
                'astra_mini.launch.py'
            )
        )
    )

    aruco_node = Node(
        package='aruco_detector',
        executable='aruco_node',
        name='aruco_detector',
        output='screen'
    )

    controller_node = Node(
        package='aruco_detector',
        executable='marker_id_controller.py',
        name='marker_position_controller',
        output='screen'
    )

    return LaunchDescription([
        astra_launch,

        TimerAction(
            period=3.0,
            actions=[aruco_node]
        ),

        TimerAction(
            period=5.0,
            actions=[controller_node]
        )
    ])
