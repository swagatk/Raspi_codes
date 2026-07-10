from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/pirobot2/cmd_vel',
        description='Twist topic used by obstacle avoidance and serial bridge',
    )

    ultrasonic_topic_arg = DeclareLaunchArgument(
        'ultrasonic_topic',
        default_value='/pirobot2/ultrasonic',
        description='Ultrasonic topic produced by serial bridge and consumed by obstacle avoidance',
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
        description='Bridge serial polling frequency for incoming sensor lines',
    )

    keepalive_arg = DeclareLaunchArgument(
        'command_keepalive_sec',
        default_value='0.5',
        description='Interval to resend current motion command so Arduino watchdog does not stop motors',
    )

    cmd_vel_timeout_arg = DeclareLaunchArgument(
        'cmd_vel_timeout_sec',
        default_value='0.8',
        description='How long to wait for cmd_vel before forcing a stop',
    )

    deadband_arg = DeclareLaunchArgument(
        'deadband',
        default_value='0.02',
        description='Deadband used by bridge when mapping Twist to motion commands',
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

    control_hz_arg = DeclareLaunchArgument(
        'control_hz',
        default_value='10.0',
        description='Obstacle avoidance control loop frequency',
    )

    sensor_timeout_arg = DeclareLaunchArgument(
        'sensor_timeout_sec',
        default_value='0.6',
        description='How long to wait for ultrasonic data before commanding stop',
    )

    obstacle_distance_arg = DeclareLaunchArgument(
        'obstacle_distance_cm',
        default_value='35',
        description='Start turning when center distance is below this threshold',
    )

    danger_distance_arg = DeclareLaunchArgument(
        'danger_distance_cm',
        default_value='20',
        description='Emergency threshold for strong avoidance behavior',
    )

    side_clearance_arg = DeclareLaunchArgument(
        'side_clearance_cm',
        default_value='18',
        description='Side obstacle threshold used to bias turning direction',
    )

    forward_speed_arg = DeclareLaunchArgument(
        'forward_speed',
        default_value='0.18',
        description='Forward speed command for clear path motion',
    )

    turn_speed_arg = DeclareLaunchArgument(
        'turn_speed',
        default_value='0.22',
        description='Angular speed command used while avoiding obstacles',
    )

    reverse_speed_arg = DeclareLaunchArgument(
        'reverse_speed',
        default_value='0.10',
        description='Reverse speed used when robot is boxed in by close obstacles',
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
            'cmd_vel_timeout_sec': LaunchConfiguration('cmd_vel_timeout_sec'),
            'deadband': LaunchConfiguration('deadband'),
            'speed_1_max': LaunchConfiguration('speed_1_max'),
            'speed_2_max': LaunchConfiguration('speed_2_max'),
            'default_speed_command': LaunchConfiguration('default_speed_command'),
        }],
    )

    obstacle_avoidance_node = Node(
        package='pirobot2',
        executable='obstacle_avoidance_node',
        name='obstacle_avoidance_node',
        output='screen',
        parameters=[{
            'ultrasonic_topic': LaunchConfiguration('ultrasonic_topic'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'control_hz': LaunchConfiguration('control_hz'),
            'sensor_timeout_sec': LaunchConfiguration('sensor_timeout_sec'),
            'obstacle_distance_cm': LaunchConfiguration('obstacle_distance_cm'),
            'danger_distance_cm': LaunchConfiguration('danger_distance_cm'),
            'side_clearance_cm': LaunchConfiguration('side_clearance_cm'),
            'forward_speed': LaunchConfiguration('forward_speed'),
            'turn_speed': LaunchConfiguration('turn_speed'),
            'reverse_speed': LaunchConfiguration('reverse_speed'),
        }],
    )

    return LaunchDescription([
        cmd_vel_topic_arg,
        ultrasonic_topic_arg,
        serial_port_arg,
        baudrate_arg,
        serial_timeout_arg,
        poll_hz_arg,
        keepalive_arg,
        cmd_vel_timeout_arg,
        deadband_arg,
        speed_1_max_arg,
        speed_2_max_arg,
        default_speed_command_arg,
        control_hz_arg,
        sensor_timeout_arg,
        obstacle_distance_arg,
        danger_distance_arg,
        side_clearance_arg,
        forward_speed_arg,
        turn_speed_arg,
        reverse_speed_arg,
        serial_bridge_node,
        obstacle_avoidance_node,
    ])
