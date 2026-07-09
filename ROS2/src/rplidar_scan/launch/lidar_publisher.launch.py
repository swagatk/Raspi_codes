from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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

    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Topic used to publish raw LaserScan data',
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Serial baud rate used by the RPLidar',
    )

    lidar_node = Node(
        package='rplidar_scan',
        executable='rplidar_scan_node',
        name='rplidar_scan_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'frame_id': LaunchConfiguration('frame_id'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'baudrate': LaunchConfiguration('baudrate'),
        }],
    )

    return LaunchDescription([
        port_arg,
        frame_id_arg,
        scan_topic_arg,
        baudrate_arg,
        lidar_node,
    ])