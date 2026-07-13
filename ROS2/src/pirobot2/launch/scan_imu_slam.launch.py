from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory('pirobot2')
    default_rviz_config = os.path.join(package_share, 'rviz', 'slam_map.rviz')

    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='LaserScan topic used by the SLAM node',
    )

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu/data_raw',
        description='IMU topic used by the SLAM node',
    )

    map_topic_arg = DeclareLaunchArgument(
        'map_topic',
        default_value='/map',
        description='OccupancyGrid map output topic',
    )

    deployment_arg = DeclareLaunchArgument(
        'deployment',
        default_value='robot',
        description='Deployment profile: robot or remote. In remote mode local lidar/imu publishers are not started.',
    )

    start_lidar_arg = DeclareLaunchArgument(
        'start_lidar',
        default_value='true',
        description='Start internal lidar publisher node',
    )

    start_imu_arg = DeclareLaunchArgument(
        'start_imu',
        default_value='true',
        description='Start internal IMU publisher node',
    )

    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Publish map->base_link transform from SLAM node',
    )

    start_static_tf_arg = DeclareLaunchArgument(
        'start_static_tf',
        default_value='true',
        description='Start static TF publishers for base_link->laser and base_link->imu_link',
    )

    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='true',
        description='Start RViz with slam map configuration',
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to RViz config for map visualization',
    )

    lidar_node = Node(
        package='pirobot2',
        executable='lidar_publisher_node',
        name='lidar_publisher_node',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('deployment'), "' == 'robot' and '", LaunchConfiguration('start_lidar'), "' != 'false'"
        ])),
        parameters=[{
            'scan_topic': LaunchConfiguration('scan_topic'),
        }],
    )

    imu_node = Node(
        package='pirobot2',
        executable='imu_publisher_node',
        name='imu_publisher_node',
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('deployment'), "' == 'robot' and '", LaunchConfiguration('start_imu'), "' != 'false'"
        ])),
        parameters=[{
            'topic': LaunchConfiguration('imu_topic'),
        }],
    )

    remote_mode_notice = LogInfo(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('deployment'), "' == 'remote'"])),
        msg='scan_imu_slam: deployment=remote, local lidar/imu publishers are disabled. Expect external /scan and /imu/data_raw publishers.',
    )

    lidar_disabled_notice = LogInfo(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('start_lidar'), "' == 'false'"])),
        msg='scan_imu_slam: start_lidar=false, no local /scan publisher will be started.',
    )

    imu_disabled_notice = LogInfo(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('start_imu'), "' == 'false'"])),
        msg='scan_imu_slam: start_imu=false, no local /imu/data_raw publisher will be started.',
    )

    laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_static_tf')),
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser',
        ],
    )

    imu_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_tf',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_static_tf')),
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu_link',
        ],
    )

    slam_node = Node(
        package='pirobot2',
        executable='scan_imu_slam_node',
        name='scan_imu_slam_node',
        output='screen',
        parameters=[{
            'scan_topic': LaunchConfiguration('scan_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'map_topic': LaunchConfiguration('map_topic'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'map_frame': 'map',
            'base_frame': 'base_link',
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='pirobot2_slam_rviz',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_rviz')),
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    return LaunchDescription([
        scan_topic_arg,
        imu_topic_arg,
        map_topic_arg,
        deployment_arg,
        start_lidar_arg,
        start_imu_arg,
        publish_tf_arg,
        start_static_tf_arg,
        start_rviz_arg,
        rviz_config_arg,
        remote_mode_notice,
        lidar_disabled_notice,
        imu_disabled_notice,
        lidar_node,
        imu_node,
        laser_tf_node,
        imu_tf_node,
        slam_node,
        rviz_node,
    ])