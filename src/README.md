# Overview
This folder holds the C++ code for Kubo, the lab's autonomous tractor.

## 1. Nodes

<!-- ********************Camera******************** -->
### 1.1 Camera

#### 1.1.1 Subscribed Topics
ROS node that takes in a black and white image and filters out values that are not in the range between _low_filter_ and _high_filter_
- __*/camera/usb_cam1/image_raw*__ ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))<br/>
Black and white image to be modified and displayed

#### 1.1.2 Parameters
- __*low_filter*__ (int, default: 0)<br/>
The low threshold for the image
- __*high_filter*__ (int, default: 0)<br/>
The high threshold for the image
- __*filter*__ (bool, default: true)<br/>
Filter the image?


<!-- ********************DriveState******************** -->
### 1.2 DriveState
ROS node that publishes an AckermannDrive message passed on from teloperation and autonomous input based on a boolean

#### 1.2.1 Published Topics
- __*drive*__ ([ackermann_msgs/AckermannDrive](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDrive.html))<br/>
Forwarded drive message

#### 1.2.2 Subscribed Topics
- __*auto*__ ([std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html))<br/>
Sets the published data to teleoperation or autonomous
- __*autodrive*__ ([ackermann_msgs/AckermannDrive](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDrive.html))<br/>
Autonomous input to be forwarded
- __*teledrive*__ ([ackermann_msgs/AckermannDrive](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDrive.html))<br/>
Teleoperation input to be forwarded


<!-- ********************Hemisphere******************** -->
### 1.3 Hemisphere
ROS driver node that publishes true heading from the Hemisphere gps

#### 1.3.1 Published Topics
- __*heading*__ ([gravl/Hemisphere](https://github.com/olinrobotics/gravl/blob/master/msg/Hemisphere.msg))<br/>
The true heading of the Hemisphere gps


<!-- ********************gps_map******************** -->
### 1.4 gps_map
ROS node for showing the position/heading on a map using Qt

#### 1.4.1 Subscribed Topics
- __*/gps/fix*__ ([sensor_msgs/NavSatFix](http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html))<br/>
GPS position of the tractor
- __*heading*__ ([gravl/Hemisphere](https://github.com/olinrobotics/gravl/blob/master/msg/Hemisphere.msg))<br/>
The true heading of the Hemisphere gps


<!-- ********************Teleop******************** -->
### 1.5 Teleop
ROS node for teleoperation of ackermann steering vehicles

#### 1.5.1 Published Topics
- __*auto*__ ([std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html))<br/>
Sets the published data to teleoperation or autonomous
- __*softestop*__ ([std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html))<br/>
Software estop
- __*teledrive*__ ([ackermann_msgs/AckermannDrive](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDrive.html))<br/>
Teleoperation output

#### 1.5.2 Subscribed Topics
- __*joy*__ ([sensor_msgs/Joy](http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html))<br/>
Gamepad input for teleoperation


## Misc stuff

### hind_brain
This folder contains Arduino code for Kubo's hind brain, running on an Arduino
Teensy. The code is run upon startup of Kubo's electronics; to interface with
the code, follow the tractor startup instructions on the home page of the Github
gravl wiki.

### `road_detection.cpp`
Takes a single image file of a road and theoretically draws a line of the direction to go in on an image.
To run with an image file named road.jpg:
```
cd scripts
cmake .
make
./road_detection road.jpg
```

### `hello_world.cpp`
Basic c++ hello world program