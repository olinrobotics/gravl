# Overview
Folder for GRAVL launch files

## Launch Files
### `bringup_min.launch`
Launches minimal tractor functionality, no map-level reference
- localization
- serial (teensy)
- teleoperation via joystick
- main front lidar
- State controller

### `bringup.launch`
Launches tractor with full environment reference, mapping capability
- all functionality in bringup_min.launch
- rtk (GPS)

### `hindbrain.launch`
Launches hindbrain ROS node
- serial (teensy)

### `ir.launch`
Launches usb_cam1 node for infrared camera, contains parameters for calibrating
camera

### `lidar.launch`
Launches hokuyo node for main lidar

### `lidarfollow.launch`
Launches minimal tractor and lidarfollow behavior


### `localization.launch`
Launches static and kinetic tf frames for Kubo localization
- robot_localization (ekf_localization_node)
- temp_tf_broadcaster
- map -> world
- lidar -> base
- lidar2 -> base
- IMU -> base
- RTK GPS -> base

### `rtk.launch`
Launches swiftnav RTK GPS

### `safety.launch`
Launches tf localization, imu, and safety handling node

### `teleop.launch`
Launches nodes for reading and publishing joystick commands
- joystick (joy_node)
- joystick_teleop
- Default controller: gamepad

### `telewaypoint.launch`
TODO

### `waypoint.launch`
launches gps waypoint converter and RTK GPS

### `telewaypoint.launch`
launches gps waypoint navigator along with fully gps-referenced tractor
