from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory('pirobot2')
    default_params = os.path.join(package_share, 'config', 'slam_toolbox_mapper_params.yaml')
    default_rviz_config = os.path.join(package_share, 'rviz', 'slam_toolbox_map.rviz')

    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/pirobot2/scan',
        description='LaserScan topic consumed by slam_toolbox',
    )

    map_topic_arg = DeclareLaunchArgument(
        'map_topic',
        default_value='/pirobot2/map',
        description='Map topic exposed to the pirobot2 ecosystem',
    )

    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value='map',
        description='Global map frame id',
    )

    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame id used internally by slam_toolbox',
    )

    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Robot base frame id',
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='slam_toolbox YAML parameter file',
    )

    debug_logging_arg = DeclareLaunchArgument(
        'debug_logging',
        default_value='false',
        description='Enable verbose slam_toolbox logging',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true',
    )

    start_static_tf_arg = DeclareLaunchArgument(
        'start_static_tf',
        default_value='true',
        description='Start base_link->laser static TF locally if your robot does not publish it',
    )

    laser_frame_arg = DeclareLaunchArgument(
        'laser_frame',
        default_value='laser',
        description='Laser frame id for static TF publisher',
    )

    laser_x_arg = DeclareLaunchArgument(
        'laser_x',
        default_value='0.0',
        description='Laser X offset from base frame in meters',
    )

    laser_y_arg = DeclareLaunchArgument(
        'laser_y',
        default_value='0.0',
        description='Laser Y offset from base frame in meters',
    )

    laser_z_arg = DeclareLaunchArgument(
        'laser_z',
        default_value='0.0',
        description='Laser Z offset from base frame in meters',
    )

    laser_roll_arg = DeclareLaunchArgument(
        'laser_roll',
        default_value='0.0',
        description='Laser roll offset from base frame in radians',
    )

    laser_pitch_arg = DeclareLaunchArgument(
        'laser_pitch',
        default_value='0.0',
        description='Laser pitch offset from base frame in radians',
    )

    laser_yaw_arg = DeclareLaunchArgument(
        'laser_yaw',
        default_value='0.0',
        description='Laser yaw offset from base frame in radians',
    )

    start_identity_odom_tf_arg = DeclareLaunchArgument(
        'start_identity_odom_tf',
        default_value='false',
        description='Start an identity odom->base_link TF when no wheel odometry publisher exists',
    )

    start_cmdvel_odom_arg = DeclareLaunchArgument(
        'start_cmdvel_odom',
        default_value='true',
        description='Start cmd_vel-integrated odometry publisher (recommended when wheel odometry is unavailable)',
    )

    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/pirobot2/cmd_vel',
        description='Twist topic consumed by cmd_vel odometry node',
    )

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Odometry topic published by cmd_vel odometry node',
    )

    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='true',
        description='Start RViz with map config',
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically configure and activate the slam_toolbox lifecycle node',
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to RViz configuration file',
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='pirobot2_slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'scan_topic': LaunchConfiguration('scan_topic'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_frame': LaunchConfiguration('map_frame'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'debug_logging': LaunchConfiguration('debug_logging'),
            },
        ],
        remappings=[
            ('scan', LaunchConfiguration('scan_topic')),
            ('/scan', LaunchConfiguration('scan_topic')),
            ('map', LaunchConfiguration('map_topic')),
            ('/map', LaunchConfiguration('map_topic')),
        ],
    )

    laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf_for_slam_toolbox',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_static_tf')),
        arguments=[
            '--x', LaunchConfiguration('laser_x'),
            '--y', LaunchConfiguration('laser_y'),
            '--z', LaunchConfiguration('laser_z'),
            '--roll', LaunchConfiguration('laser_roll'),
            '--pitch', LaunchConfiguration('laser_pitch'),
            '--yaw', LaunchConfiguration('laser_yaw'),
            '--frame-id', LaunchConfiguration('base_frame'),
            '--child-frame-id', LaunchConfiguration('laser_frame'),
        ],
    )

    odom_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_tf_for_slam_toolbox',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_identity_odom_tf')),
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', LaunchConfiguration('odom_frame'),
            '--child-frame-id', LaunchConfiguration('base_frame'),
        ],
    )

    cmdvel_odom_node = Node(
        package='pirobot2',
        executable='cmdvel_odom_node',
        name='cmdvel_odom_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_cmdvel_odom')),
        parameters=[{
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'publish_tf': True,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='pirobot2_slam_toolbox_rviz',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_rviz')),
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    configure_slam_toolbox = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/pirobot2_slam_toolbox', 'configure'],
                output='screen',
                condition=IfCondition(LaunchConfiguration('autostart')),
            ),
        ],
    )

    activate_slam_toolbox = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/pirobot2_slam_toolbox', 'activate'],
                output='screen',
                condition=IfCondition(LaunchConfiguration('autostart')),
            ),
        ],
    )

    return LaunchDescription([
        scan_topic_arg,
        map_topic_arg,
        map_frame_arg,
        odom_frame_arg,
        base_frame_arg,
        params_file_arg,
        debug_logging_arg,
        use_sim_time_arg,
        start_static_tf_arg,
        laser_frame_arg,
        laser_x_arg,
        laser_y_arg,
        laser_z_arg,
        laser_roll_arg,
        laser_pitch_arg,
        laser_yaw_arg,
        start_identity_odom_tf_arg,
        start_cmdvel_odom_arg,
        cmd_vel_topic_arg,
        odom_topic_arg,
        start_rviz_arg,
        autostart_arg,
        rviz_config_arg,
        slam_toolbox_node,
        laser_tf_node,
        odom_tf_node,
        cmdvel_odom_node,
        configure_slam_toolbox,
        activate_slam_toolbox,
        rviz_node,
    ])
