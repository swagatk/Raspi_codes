from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/pirobot2/cmd_vel',
        description='Twist command topic consumed by the serial bridge',
    )

    ultrasonic_topic_arg = DeclareLaunchArgument(
        'ultrasonic_topic',
        default_value='/pirobot2/ultrasonic',
        description='Topic used to publish [left, center, right] ultrasonic values',
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Arduino serial device path',
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='115200',
        description='Arduino serial baud rate',
    )

    serial_timeout_arg = DeclareLaunchArgument(
        'serial_timeout',
        default_value='0.1',
        description='Serial read/write timeout (seconds)',
    )

    poll_hz_arg = DeclareLaunchArgument(
        'poll_hz',
        default_value='50.0',
        description='Serial polling frequency for incoming sensor lines',
    )

    deadband_arg = DeclareLaunchArgument(
        'deadband',
        default_value='0.02',
        description='Deadband used to map Twist commands to stop/drive characters',
    )

    speed_1_max_arg = DeclareLaunchArgument(
        'speed_1_max',
        default_value='0.12',
        description='Max Twist magnitude that maps to Arduino speed command 1',
    )

    speed_2_max_arg = DeclareLaunchArgument(
        'speed_2_max',
        default_value='0.22',
        description='Max Twist magnitude that maps to Arduino speed command 2',
    )

    default_speed_command_arg = DeclareLaunchArgument(
        'default_speed_command',
        default_value='2',
        description='Initial Arduino speed command to send at startup (1/2/3)',
    )

    serial_bridge_node = Node(
        package='pirobot2',
        executable='serial_bridge_node',
        name='serial_bridge_node',
        output='screen',
        parameters=[{
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'ultrasonic_topic': LaunchConfiguration('ultrasonic_topic'),
            'serial_port': LaunchConfiguration('serial_port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'serial_timeout': LaunchConfiguration('serial_timeout'),
            'poll_hz': LaunchConfiguration('poll_hz'),
            'deadband': LaunchConfiguration('deadband'),
            'speed_1_max': LaunchConfiguration('speed_1_max'),
            'speed_2_max': LaunchConfiguration('speed_2_max'),
            'default_speed_command': LaunchConfiguration('default_speed_command'),
        }],
    )

    return LaunchDescription([
        cmd_vel_topic_arg,
        ultrasonic_topic_arg,
        serial_port_arg,
        baudrate_arg,
        serial_timeout_arg,
        poll_hz_arg,
        deadband_arg,
        speed_1_max_arg,
        speed_2_max_arg,
        default_speed_command_arg,
        serial_bridge_node,
    ])
