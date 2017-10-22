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

#### 'LidarCodeBasic.py'
Takes a scan topic from LIDAR to track obstacles. To run, first either run a node that produces a scan topic (e.g. rosrun urg_node urg_node) or the bags labeled tractorLidar*. Open rviz with rosrun rviz rviz. Press the + button and add by topic /scan. Change the box that has 'map' to '/laser'. You should be able to visualize the data. Now run the program (rosrun gravl LidarCodeBasic.py) and data will be published. /estop is whether there is an obstacle in the way, /scan_verticals is the distance from the tractor of the obstacle, /scan_horzontals is the distance horizontally from the center of the tractor.
