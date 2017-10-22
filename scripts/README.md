### Overview
This folder holds general scripts for GRAVL

### Files
#### `test_pointgrey.py`
Displays raw imagery from a pointgrey camera publishing through ROS. To run, first setup a pointgrey camera such that it is
publishing over ROS by looking at this wiki page: ([link](https://github.com/olinrobotics/gravl/wiki/Kubo:-Cameras))  Then,
run the pointgrey program by entering `python pointgrey.py` in the Terminal.

#### `pointgrey_bag.py`
Displays bag file of pointgrey camera imagery. To run, first setup a pointgrey camera such that it is
publishing over ROS by looking at this wiki page: ([link](https://github.com/olinrobotics/gravl/wiki/Kubo:-Cameras))  Then, 
run the program by entering `python pointgrey_bag.py "filename"`, where `"filename"` is the name of a rosbag file of the 
`/camera/image_raw` topic.
