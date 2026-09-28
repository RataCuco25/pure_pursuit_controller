import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch Gazebo and the Pure Pursuit controller."""

    lookahead_gain = LaunchConfiguration(
        'lookahead_gain'
    )

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
        parameters=[
            {
                'lookahead_gain': lookahead_gain,
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'lookahead_gain',
            default_value='1.0',
            description='Pure Pursuit lookahead gain k',
        ),
        prius_launch,
        pure_pursuit_node,
    ])
