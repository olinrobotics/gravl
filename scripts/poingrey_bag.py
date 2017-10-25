import sys
import rosbag
import cv2
import time
from std_msgs.msg import Int32, String
from cv_bridge import CvBridge, CvBridgeError

# if you have bag name as a command line argument.
if len(sys.argv) > 1:
    # make the bridge
    bridge = CvBridge()
    # get the data from the specified bag file
    bagname = sys.argv[1]
    bag = rosbag.Bag(bagname)
    # iterate through the messages and display them
    for topic, msg, t in bag.read_messages(topics=['/camera/image_raw']):
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("Image raw", cv_image)
        cv2.waitKey(1)
    # close the bag
    bag.close()
# you done fucked up.
else:
    print("Please specify a bag file as an argument.")

