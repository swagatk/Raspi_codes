from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/pirobot2/cmd_vel',
        description='Twist topic used between teleop and serial bridge',
    )

    ultrasonic_topic_arg = DeclareLaunchArgument(
        'ultrasonic_topic',
        default_value='/pirobot2/ultrasonic',
        description='Topic used by serial bridge to publish ultrasonic readings',
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
        description='Serial polling loop frequency used by the bridge',
    )

    keepalive_arg = DeclareLaunchArgument(
        'command_keepalive_sec',
        default_value='0.5',
        description='Interval to resend current motion command for Arduino watchdog',
    )

    deadband_arg = DeclareLaunchArgument(
        'deadband',
        default_value='0.02',
        description='Deadband for linear/angular command to stop jitter',
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

    speed_1_arg = DeclareLaunchArgument(
        'speed_1',
        default_value='0.10',
        description='Teleop linear speed for keyboard profile 1 (m/s)',
    )

    speed_2_arg = DeclareLaunchArgument(
        'speed_2',
        default_value='0.20',
        description='Teleop linear speed for keyboard profile 2 (m/s)',
    )

    speed_3_arg = DeclareLaunchArgument(
        'speed_3',
        default_value='0.30',
        description='Teleop linear speed for keyboard profile 3 (m/s)',
    )

    angular_scale_arg = DeclareLaunchArgument(
        'angular_scale',
        default_value='1.0',
        description='Multiplier applied to teleop turning speed',
    )

    idle_timeout_arg = DeclareLaunchArgument(
        'idle_timeout_sec',
        default_value='0.4',
        description='Auto-stop delay if no motion key is pressed',
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
            'command_keepalive_sec': LaunchConfiguration('command_keepalive_sec'),
            'deadband': LaunchConfiguration('deadband'),
            'speed_1_max': LaunchConfiguration('speed_1_max'),
            'speed_2_max': LaunchConfiguration('speed_2_max'),
            'default_speed_command': LaunchConfiguration('default_speed_command'),
        }],
    )

    teleop_keyboard_node = Node(
        package='pirobot2',
        executable='teleop_keyboard_node',
        name='teleop_keyboard_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'topic': LaunchConfiguration('cmd_vel_topic'),
            'speed_1': LaunchConfiguration('speed_1'),
            'speed_2': LaunchConfiguration('speed_2'),
            'speed_3': LaunchConfiguration('speed_3'),
            'angular_scale': LaunchConfiguration('angular_scale'),
            'idle_timeout_sec': LaunchConfiguration('idle_timeout_sec'),
        }],
    )

    return LaunchDescription([
        topic_arg,
        ultrasonic_topic_arg,
        serial_port_arg,
        baudrate_arg,
        serial_timeout_arg,
        poll_hz_arg,
        keepalive_arg,
        deadband_arg,
        speed_1_max_arg,
        speed_2_max_arg,
        default_speed_command_arg,
        speed_1_arg,
        speed_2_arg,
        speed_3_arg,
        angular_scale_arg,
        idle_timeout_arg,
        serial_bridge_node,
        teleop_keyboard_node,
    ])
