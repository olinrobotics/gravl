#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2

def img_callback(data):
    ''' DOCSTRING:
        Given img data from usb cam, saves img for later use; called every
        time img recieved from usb cam
        '''
    try:
        global cv_image
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

    except CvBridgeError as e:
        print('ERR:%d',e)

bridge = CvBridge()
cv_image = None

rospy.init_node('test_cam1')
rospy.Subscriber('/camera/image_raw', Image, img_callback) #Subscribes to camera feed; calls img_callback upon receipt of image

r = rospy.Rate(20)
while not rospy.is_shutdown():
    r.sleep()
    if (cv_image != None):
        cv2.imshow('Image_raw', cv_image)
        cv2.waitKey(1)
