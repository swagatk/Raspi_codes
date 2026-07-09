from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory('rplidar_scan')
    default_rviz_config = os.path.join(package_share, 'rviz', 'rplidar_scan.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rplidar_rviz',
        output='screen',
        arguments=['-d', default_rviz_config],
    )

    return LaunchDescription([
        rviz_node,
    ])