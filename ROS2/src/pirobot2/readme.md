# pirobot2 ROS 2 Package

This package provides motion control, Arduino serial bridging, ultrasonic obstacle avoidance, LiDAR and IMU publishers, RViz clients, and a lightweight LiDAR + IMU SLAM node for PiRobot2.

Tested in this workspace with ROS 2 Jazzy and a Raspberry Pi robot.

## 1) Package Contents

Main launch files:

- launch/motion_teleop.launch.py
- launch/serial_bridge.launch.py
- launch/obstacle_avoidance.launch.py
- launch/lidar_publisher.launch.py
- launch/imu_publisher.launch.py
- launch/lidar_rviz_client.launch.py
- launch/lidar_imu_rviz_client.launch.py
- launch/scan_imu_slam.launch.py

Main nodes (ros2 run pirobot2 ...):

- teleop_keyboard_node
- serial_bridge_node
- obstacle_avoidance_node
- lidar_publisher_node
- imu_publisher_node
- scan_imu_slam_node
- motion_subscriber_node
- ultrasonic_publisher_node

## 2) Prerequisites

System dependencies:

- ROS 2 Jazzy installed and sourced
- rviz2
- slam-toolbox (recommended for production maps)
- pyserial
- rplidar (Python package)
- smbus2 (or smbus)

Install slam-toolbox (Ubuntu/ROS 2 apt):

	sudo apt install ros-jazzy-slam-toolbox

Install useful Python dependencies:

	python3 -m pip install pyserial rplidar smbus2

On Raspberry Pi, ensure serial/I2C access is enabled and user has permissions:

	sudo usermod -aG dialout $USER

Log out/in after changing group membership.

## 3) Build and Source

From workspace root (example: ROS2):

	source /opt/ros/jazzy/setup.bash
	colcon build --packages-select pirobot2 --symlink-install
	source install/setup.bash

## 4) Common Topics

- /pirobot2/cmd_vel (geometry_msgs/Twist)
- /pirobot2/ultrasonic (std_msgs/Int32MultiArray [left, center, right] cm)
- /pirobot2/scan (sensor_msgs/LaserScan)
- /pirobot2/imu/data_raw (sensor_msgs/Imu)
- /pirobot2/map (nav_msgs/OccupancyGrid)
- /pirobot2/slam_pose (geometry_msgs/PoseStamped)
- /pirobot2/slam_path (nav_msgs/Path)
- /tf, /tf_static

Quick checks:

	ros2 topic list
	ros2 topic info /pirobot2/scan -v
	ros2 topic echo /pirobot2/scan --qos-reliability best_effort --once
	ros2 topic echo /pirobot2/imu/data_raw --once

## 5) Motion Control

### 5.1 Teleop + serial bridge (manual driving)

	ros2 launch pirobot2 motion_teleop.launch.py

Default keyboard controls:

- w forward
- s backward
- a turn left
- d or b turn right
- 1/2/3 speed profile
- space stop
- q quit

Useful overrides:

	ros2 launch pirobot2 motion_teleop.launch.py serial_port:=/dev/ttyACM0 baudrate:=115200

Remote teleop (serial bridge already running on robot):

	ros2 launch pirobot2 motion_teleop.launch.py start_serial_bridge:=false

### 5.2 Serial bridge only

	ros2 launch pirobot2 serial_bridge.launch.py

This bridges /pirobot2/cmd_vel to Arduino motion chars and publishes ultrasonic distances on /pirobot2/ultrasonic.

## 6) Obstacle Avoidance

Run autonomous obstacle avoidance (uses ultrasonic from serial bridge):

	ros2 launch pirobot2 obstacle_avoidance.launch.py

Common tuning examples:

	ros2 launch pirobot2 obstacle_avoidance.launch.py obstacle_distance_cm:=40 danger_distance_cm:=22 turn_speed:=0.25

## 7) LiDAR and IMU Data

### 7.1 LiDAR publisher

	ros2 launch pirobot2 lidar_publisher.launch.py

Typical custom port:

	ros2 launch pirobot2 lidar_publisher.launch.py port:=/dev/ttyUSB0 baudrate:=115200 frame_id:=laser

### 7.2 IMU publisher (MPU6050)

	ros2 launch pirobot2 imu_publisher.launch.py

Typical custom bus/rate:

	ros2 launch pirobot2 imu_publisher.launch.py bus_num:=1 rate_hz:=50.0 frame_id:=imu_link

### 7.3 RViz sensor views

LiDAR only:

	ros2 launch pirobot2 lidar_rviz_client.launch.py

LiDAR + IMU:

	ros2 launch pirobot2 lidar_imu_rviz_client.launch.py

## 8) SLAM

The current scan_imu_slam node uses:

- LiDAR scan matching as primary XY motion estimator
- IMU for yaw prior and stationary detection
- Occupancy grid map updates from scans

### 8.1 Local SLAM on robot (starts local sensors by default)

	ros2 launch pirobot2 scan_imu_slam.launch.py deployment:=robot

### 8.2 Remote SLAM client (robot publishes sensors, remote only maps)

Run sensor publishers on robot:

	ros2 launch pirobot2 lidar_publisher.launch.py
	ros2 launch pirobot2 imu_publisher.launch.py

Run SLAM on remote machine:

	ros2 launch pirobot2 scan_imu_slam.launch.py deployment:=remote start_static_tf:=false

If robot does not publish static transforms for laser/imu frames, set start_static_tf:=true on the machine running SLAM.

### 8.3 Important SLAM launch arguments

Core:

- deployment: robot or remote
- start_lidar: true/false
- start_imu: true/false
- start_static_tf: true/false
- start_rviz: true/false
- publish_tf: true/false

Scan matching:

- scan_match_enabled
- scan_match_window_m
- scan_match_step_m
- scan_match_min_points
- scan_match_prior_weight

IMU stabilization:

- yaw_filter_window
- zupt_accel_threshold
- zupt_gyro_threshold
- zupt_min_samples

Example tuned run:

	ros2 launch pirobot2 scan_imu_slam.launch.py deployment:=remote start_static_tf:=false scan_match_window_m:=0.25 scan_match_step_m:=0.05 scan_match_prior_weight:=0.25 yaw_filter_window:=7 zupt_accel_threshold:=0.20 zupt_gyro_threshold:=0.06 zupt_min_samples:=12

### 8.4 Recommended upgrade: slam_toolbox (loop closure)

The lightweight `scan_imu_slam` node is useful for quick experiments but does not perform full global loop-closure optimization. For cleaner maps after revisiting the same area, prefer `slam_toolbox`.

Run sensor publishers on robot:

	ros2 launch pirobot2 lidar_publisher.launch.py
	ros2 launch pirobot2 imu_publisher.launch.py

Run teleop on remote laptop (without local serial bridge):

	ros2 launch pirobot2 motion_teleop.launch.py start_serial_bridge:=false

Run slam_toolbox on remote laptop:

	ros2 launch pirobot2 slam_toolbox.launch.py

If the node starts but never publishes `/pirobot2/map`, rerun once with debug logs enabled:

	ros2 launch pirobot2 slam_toolbox.launch.py start_rviz:=false debug_logging:=true

Defaults are chosen for robots without wheel odometry:

- `odom_frame:=odom`
- `start_static_tf:=true`
- `start_identity_odom_tf:=true`

If your robot already publishes `base_link -> laser` static TF, set `start_static_tf:=false`.

If your robot already publishes `odom -> base_link`, set `start_identity_odom_tf:=false`.

If the laser is not exactly at the robot origin, pass its pose explicitly, for example:

	ros2 launch pirobot2 slam_toolbox.launch.py laser_x:=0.08 laser_y:=0.0 laser_yaw:=0.0

Parameters are in:

	config/slam_toolbox_mapper_params.yaml

You can tune this file gradually (start with `map_update_interval`, `minimum_travel_distance`, `minimum_travel_heading`, and `loop_search_maximum_distance`).

If RViz shows `Message Filter dropping message ... queue is full` for frame `laser`, the TF chain is incomplete. Check that:

- `odom -> base_link` exists
- `base_link -> laser` exists
- `map -> odom` is being published by slam_toolbox
- the RViz fixed frame is `map`

Useful checks:

	ros2 run tf2_ros tf2_echo odom base_link
	ros2 run tf2_ros tf2_echo base_link laser
	ros2 run tf2_ros tf2_echo map odom

### 8.5 Cartographer and Hector notes

- Cartographer can improve global consistency, but setup and tuning effort are higher than slam_toolbox.
- Hector SLAM can work for lidar-only use cases, but ROS 2 Jazzy support is less straightforward than slam_toolbox.
- Recommended path: stabilize with slam_toolbox first, then evaluate Cartographer if needed.

## 9) Multi-machine Notes (Robot + Remote)

Set matching middleware environment on both machines:

	export ROS_DOMAIN_ID=23
	export ROS_LOCALHOST_ONLY=0

Optional explicit RMW (if needed on both):

	export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

Then source ROS/workspace on both and verify remote sees robot topics:

	ros2 topic list
	ros2 topic info /pirobot2/scan -v

Important: if /pirobot2/scan does not publish on robot itself, remote SLAM cannot work.

## 10) Troubleshooting

1) /pirobot2/scan shows subscribers but publisher count is 0

- LiDAR node is not running or failed to open serial port.
- Start lidar publisher and check logs.

2) Cannot echo /pirobot2/scan

- Use sensor-data QoS:

	ros2 topic echo /pirobot2/scan --qos-reliability best_effort --once

3) IMU not publishing

- Check I2C bus and sensor wiring.
- Verify smbus2 installation.

4) RViz GLSL shader error for map display on some GPUs

- Try software rendering:

	  LIBGL_ALWAYS_SOFTWARE=1 rviz2

5) Remote machine receives no topics

- Verify ROS_DOMAIN_ID/ROS_LOCALHOST_ONLY on both machines.
- Verify robot topics work locally first.

## 11) Safety

- Keep robot lifted or in open space for first motion tests.
- Confirm emergency stop behavior (space key or stopping cmd_vel publisher).
- Validate serial port and motor direction before high-speed operation.

