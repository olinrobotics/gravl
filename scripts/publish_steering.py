#!/usr/bin/env python

#unfinished, need to use real data from bag

import rospy
from gps_navigation import steering_angle
from std_msgs.msg import Float64

def publish_steering_angle(steering_angle):
    steering_angle_command = Float64()
    steering_angle_command.data = steering_angle

    #add in steering velocity
    pub = rospy.Publisher('steering_angle', Float64, queue_size=10)
    rospy.init_node('steering_angle', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #hello_str = "this is the angle %s" % rospy.get_time() % steering_angle
        rospy.loginfo(steering_angle_command)
        pub.publish(steering_angle_command)
        rate.sleep()

if __name__=='__main__':
    try:
        publish_steering_angle(steering_angle)
    except rospy.ROSInterruptException:
        pass
