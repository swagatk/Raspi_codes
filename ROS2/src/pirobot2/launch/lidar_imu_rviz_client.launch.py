from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory('pirobot2')
    default_rviz_config = os.path.join(package_share, 'rviz', 'lidar_imu_combined.rviz')

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to rviz config file for combined lidar and IMU visualization',
    )

    fixed_frame_arg = DeclareLaunchArgument(
        'fixed_frame',
        default_value='base_link',
        description='RViz fixed frame; should match publisher frame_id or available TF frame',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='pirobot2_lidar_imu_rviz',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config'), '-f', LaunchConfiguration('fixed_frame')],
    )

    return LaunchDescription([
        rviz_config_arg,
        fixed_frame_arg,
        rviz_node,
    ])