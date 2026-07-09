from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory('pirobot2')
    default_rviz_config = os.path.join(package_share, 'rviz', 'lidar_scan.rviz')

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to rviz config file for lidar scan visualization',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='pirobot2_lidar_rviz',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    return LaunchDescription([
        rviz_config_arg,
        rviz_node,
    ])
