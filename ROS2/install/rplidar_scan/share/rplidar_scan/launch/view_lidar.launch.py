from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory('rplidar_scan')
    default_rviz_config = os.path.join(package_share, 'rviz', 'rplidar_scan.rviz')

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='USB serial port used by the RPLidar',
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser',
        description='Frame ID assigned to LaserScan messages',
    )

    lidar_node = Node(
        package='rplidar_scan',
        executable='rplidar_scan_node',
        name='rplidar_scan_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'frame_id': LaunchConfiguration('frame_id'),
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rplidar_rviz',
        output='screen',
        arguments=['-d', default_rviz_config],
    )

    return LaunchDescription([
        port_arg,
        frame_id_arg,
        lidar_node,
        rviz_node,
    ])