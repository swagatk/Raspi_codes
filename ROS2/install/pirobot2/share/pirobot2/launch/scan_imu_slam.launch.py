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
        default_value='/pirobot2/scan',
        description='LaserScan topic used by the SLAM node',
    )

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/pirobot2/imu/data_raw',
        description='IMU topic used by the SLAM node',
    )

    map_topic_arg = DeclareLaunchArgument(
        'map_topic',
        default_value='/pirobot2/map',
        description='OccupancyGrid map output topic',
    )

    tf_topic_arg = DeclareLaunchArgument(
        'tf_topic',
        default_value='/tf',
        description='TF topic to use for transforms',
    )

    tf_static_topic_arg = DeclareLaunchArgument(
        'tf_static_topic',
        default_value='/tf_static',
        description='Static TF topic to use for static transforms',
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

    scan_match_enabled_arg = DeclareLaunchArgument(
        'scan_match_enabled',
        default_value='true',
        description='Use lidar scan matching to estimate XY motion',
    )

    scan_match_window_m_arg = DeclareLaunchArgument(
        'scan_match_window_m',
        default_value='0.25',
        description='XY search window in meters for scan matching',
    )

    scan_match_step_m_arg = DeclareLaunchArgument(
        'scan_match_step_m',
        default_value='0.05',
        description='XY search step in meters for scan matching',
    )

    scan_match_min_points_arg = DeclareLaunchArgument(
        'scan_match_min_points',
        default_value='20',
        description='Minimum valid scan points required before matching',
    )

    scan_match_prior_weight_arg = DeclareLaunchArgument(
        'scan_match_prior_weight',
        default_value='0.20',
        description='Penalty weight for large XY jumps during scan matching',
    )

    yaw_filter_window_arg = DeclareLaunchArgument(
        'yaw_filter_window',
        default_value='5',
        description='Moving-average window size (samples) for IMU yaw stabilization',
    )

    zupt_accel_threshold_arg = DeclareLaunchArgument(
        'zupt_accel_threshold',
        default_value='0.15',
        description='ZUPT acceleration threshold in m/s^2 used to detect stationary state',
    )

    zupt_gyro_threshold_arg = DeclareLaunchArgument(
        'zupt_gyro_threshold',
        default_value='0.05',
        description='ZUPT angular velocity threshold in rad/s used to detect stationary state',
    )

    zupt_min_samples_arg = DeclareLaunchArgument(
        'zupt_min_samples',
        default_value='8',
        description='Consecutive IMU samples below thresholds before zeroing velocity',
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
        msg='scan_imu_slam: deployment=remote, local lidar/imu publishers are disabled. Expect external /pirobot2/scan and /pirobot2/imu/data_raw publishers.',
    )

    lidar_disabled_notice = LogInfo(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('start_lidar'), "' == 'false'"])),
        msg='scan_imu_slam: start_lidar=false, no local /pirobot2/scan publisher will be started.',
    )

    imu_disabled_notice = LogInfo(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('start_imu'), "' == 'false'"])),
        msg='scan_imu_slam: start_imu=false, no local /pirobot2/imu/data_raw publisher will be started.',
    )

    laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_static_tf')),
        remappings=[
            ('/tf', LaunchConfiguration('tf_topic')),
            ('/tf_static', LaunchConfiguration('tf_static_topic')),
        ],
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
        remappings=[
            ('/tf', LaunchConfiguration('tf_topic')),
            ('/tf_static', LaunchConfiguration('tf_static_topic')),
        ],
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
        remappings=[
            ('/tf', LaunchConfiguration('tf_topic')),
            ('/tf_static', LaunchConfiguration('tf_static_topic')),
        ],
        parameters=[{
            'scan_topic': LaunchConfiguration('scan_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'map_topic': LaunchConfiguration('map_topic'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'scan_match_enabled': LaunchConfiguration('scan_match_enabled'),
            'scan_match_window_m': LaunchConfiguration('scan_match_window_m'),
            'scan_match_step_m': LaunchConfiguration('scan_match_step_m'),
            'scan_match_min_points': LaunchConfiguration('scan_match_min_points'),
            'scan_match_prior_weight': LaunchConfiguration('scan_match_prior_weight'),
            'yaw_filter_window': LaunchConfiguration('yaw_filter_window'),
            'zupt_accel_threshold': LaunchConfiguration('zupt_accel_threshold'),
            'zupt_gyro_threshold': LaunchConfiguration('zupt_gyro_threshold'),
            'zupt_min_samples': LaunchConfiguration('zupt_min_samples'),
            'pose_topic': '/pirobot2/slam_pose',
            'path_topic': '/pirobot2/slam_path',
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
        remappings=[
            ('/tf', LaunchConfiguration('tf_topic')),
            ('/tf_static', LaunchConfiguration('tf_static_topic')),
        ],
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    return LaunchDescription([
        scan_topic_arg,
        imu_topic_arg,
        map_topic_arg,
        tf_topic_arg,
        tf_static_topic_arg,
        deployment_arg,
        start_lidar_arg,
        start_imu_arg,
        publish_tf_arg,
        scan_match_enabled_arg,
        scan_match_window_m_arg,
        scan_match_step_m_arg,
        scan_match_min_points_arg,
        scan_match_prior_weight_arg,
        yaw_filter_window_arg,
        zupt_accel_threshold_arg,
        zupt_gyro_threshold_arg,
        zupt_min_samples_arg,
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