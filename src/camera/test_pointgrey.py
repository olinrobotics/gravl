#!/urs/bin env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2

bridge = CvBridge()

def img_callback(data):
    ''' DOCSTRING:
        Given img data from usb cam, saves img for later use; called every
        time img recieved from usb cam
        '''
    try:
        curr_frame = bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.imshow('Raw Feed',curr_frame)

    except CvBridgeError as e:
        print('ERR:%d',e)

def setup():
    rospy.init_node('test_cam1') # Creates ROSnode to subscribe to camera topics
    rospy.Subscriber('/camera/image_raw', Image, img_callback) #Subscribes to camera feed; calls img_callback upon receipt of image

if __name__=='__main__':
    setup()
    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        r.sleep()
