import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Launch Gazebo and the Pure Pursuit controller."""
    prius_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('prius_bringup'),
                'launch',
                'gz_sim.launch.py'
            )
        )
    )

    pure_pursuit_node = Node(
        package='pure_pursuit_controller',
        executable='pure_pursuit_node',
        name='pure_pursuit_controller',
        output='screen',
    )

    return LaunchDescription([
        prius_launch,
        pure_pursuit_node,
    ])
