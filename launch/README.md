# Overview
This folder holds the launch files for Kubo, the lab's autonomous tractor

## Launch Files
### `bringup_min.launch`
Launches minimal basic tractor functionality
- localization
- serial (teensy)

### `bringup.launch`
Launches basic tractor functionality
- localization
- serial (teensy)
- rtk (GPS)
- teleop
- lidar (main)

### `imu.launch`
Launches phidgets imu and imu_filtering node
- imu_node
- imu_filter_node

### `ir.launch`
Launches usb_cam1 node for infrared camera

### `laser_to_pc.launch`
Launches nodes to convert laser scans to single point cloud
Copied from: http://www.theconstructsim.com/merge-laser-scans-single-pointcloud/

### `LidarFollowLaunch.launch`
TODO

### `lidar.launch`
Launches hokuyo node for main lidar

### `localization.launch`
Launches static and kinetic tf frames for Kubo localization
- robot_localization (ekf_localization_node)
- temp_tf_broadcaster
- map -> world
- gps -> base
- lidar -> base
- lidar2 -> base
- IMU -> base
- camera -> base
- hemisphere -> base
- RTK GPS -> base

### `localize_test.launch`
Launch nodes for testing localization with ekf_localization_node
- robot_localization
- imu.launch
- rtk.launch

### `rtk.launch`
Launches rtk gps node

### `teleop.launch`
Launches nodes for reading and publishing joystick commands
- joystick (joy_node)
- joystick_teleop

### `telewaypoint.launch`

### `waypoint.launch`
