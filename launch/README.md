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

### `ir.launch`
Launches usb_cam1 node for infrared camera

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

### `teleop.launch`
Launches nodes for reading and publishing joystick commands
- joystick (joy_node)
- joystick_teleop

### `telewaypoint.launch`
Launches 
