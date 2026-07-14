from glob import glob
from setuptools import find_packages, setup

package_name = 'pirobot2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='swagat.kumar@gmail.com',
    description='ROS2 control interface for PiRobot 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pirobot2 = pirobot2.pirobot2:main',
            'serial_bridge_node = pirobot2.serial_bridge:main',
            'obstacle_avoidance_node = pirobot2.obstacle_avoidance:main',
            'imu_publisher_node = pirobot2.imu_publisher:main',
            'lidar_publisher_node = pirobot2.lidar_publisher:main',
            'scan_imu_slam_node = pirobot2.scan_imu_slam:main',
            'motion_subscriber_node = pirobot2.motion_subscriber:main',
            'teleop_keyboard_node = pirobot2.teleop_keyboard:main',
            'ultrasonic_publisher_node = pirobot2.ultrasonic_publisher:main',
        ],
    },
)
