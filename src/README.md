# Overview
This folder holds the C++ code for Kubo, the lab's autonomous tractor.

## 1. Nodes

### 1.1 Camera
#### 1.1.1 Published Topics
drive

#### 1.1.2 Subscribed Topics
- auto
- autodrive
- teledrive 

### 1.2 Hemisphere
#### 1.2.1 Published Topics
drive

#### 1.2.2 Subscribed Topics
- auto
- autodrive
- teledrive 

### 1.3 State
ROS node that publishes 
#### 1.3.1 Published Topics
drive ([ackermann_msgs/AckermannDrive](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDrive.html))

#### 1.3.2 Subscribed Topics
- auto ([std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html))<br/>
hello
- autodrive ([ackermann_msgs/AckermannDrive](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDrive.html))
- teledrive ([ackermann_msgs/AckermannDrive](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDrive.html))

### 1.4 Teleop
#### 1.4.1 Published Topics
drive

#### 1.4.2 Subscribed Topics
- auto
- autodrive
- teledrive 


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