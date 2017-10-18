#!/usr/bin/env python

import rospy
from gps_navigation import *

def publish_steering_angle(steering_angle):
    #add in steering velocity
    pub = rospy.Publisher('steering_angle', String, queue_size=10)
    rospy.init_node('steering_angle', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #hello_str = "this is the angle %s" % rospy.get_time() % steering_angle
        rospy.loginfo(steering_angle)
        pub.publish(steering_angle)
        rate.sleep()

if __name__=='__main__':
    try:
        publish_steering_angle(steering_angle)
    except rospy.ROSInterruptException:
        pass
