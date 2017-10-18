#!/usr/bin/env python

import rospy
from gps_navigation import *

def publish_forward_velocity(forward_velocity):
    pub = rospy.Publisher('forward_velocity', String, queue_size=10)
    rospy.init_node('forward_velocity', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #hello_str = "this is the angle %s" % rospy.get_time() % steering_angle
        rospy.loginfo(forward_velocity)
        pub.publish(forward_velocity)
        rate.sleep()

if __name__=='__main__':
    try:
        publish_forward_velocity(forward_velocity)
    except rospy.ROSInterruptException:
        pass
