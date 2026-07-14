from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory('pirobot2')
    default_params = os.path.join(package_share, 'config', 'slam_toolbox_mapper_params.yaml')
    default_rviz_config = os.path.join(package_share, 'rviz', 'slam_map.rviz')

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

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true',
    )

    start_static_tf_arg = DeclareLaunchArgument(
        'start_static_tf',
        default_value='false',
        description='Start base_link->laser static TF locally if your robot does not publish it',
    )

    laser_frame_arg = DeclareLaunchArgument(
        'laser_frame',
        default_value='laser',
        description='Laser frame id for static TF publisher',
    )

    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='true',
        description='Start RViz with map config',
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
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_frame': LaunchConfiguration('map_frame'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
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
        arguments=['0', '0', '0', '0', '0', '0', LaunchConfiguration('base_frame'), LaunchConfiguration('laser_frame')],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='pirobot2_slam_toolbox_rviz',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_rviz')),
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    return LaunchDescription([
        scan_topic_arg,
        map_topic_arg,
        map_frame_arg,
        odom_frame_arg,
        base_frame_arg,
        params_file_arg,
        use_sim_time_arg,
        start_static_tf_arg,
        laser_frame_arg,
        start_rviz_arg,
        rviz_config_arg,
        slam_toolbox_node,
        laser_tf_node,
        rviz_node,
    ])
