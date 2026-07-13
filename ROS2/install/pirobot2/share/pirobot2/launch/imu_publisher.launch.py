from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bus_num_arg = DeclareLaunchArgument(
        'bus_num',
        default_value='1',
        description='I2C bus number used by the MPU6050',
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu_link',
        description='Frame ID used in the IMU message header',
    )

    topic_arg = DeclareLaunchArgument(
        'topic',
        default_value='/imu/data_raw',
        description='Topic to publish filtered IMU data',
    )

    rate_arg = DeclareLaunchArgument(
        'rate_hz',
        default_value='50.0',
        description='IMU publish rate in Hz',
    )

    imu_node = Node(
        package='pirobot2',
        executable='imu_publisher_node',
        name='imu_publisher_node',
        output='screen',
        parameters=[{
            'bus_num': LaunchConfiguration('bus_num'),
            'frame_id': LaunchConfiguration('frame_id'),
            'topic': LaunchConfiguration('topic'),
            'rate_hz': LaunchConfiguration('rate_hz'),
        }],
    )

    return LaunchDescription([
        bus_num_arg,
        frame_id_arg,
        topic_arg,
        rate_arg,
        imu_node,
    ])