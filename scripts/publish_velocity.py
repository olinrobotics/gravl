#!/usr/bin/env python

#unfinished, need to use real data from bag

import rospy
from gps_navigation import forward_velocity
from std_msgs.msg import Float64

def publish_forward_velocity(forward_velocity):
    forward_velocity_command = Float64()
    forward_velocity_command.data = forward_velocity

    pub = rospy.Publisher('forward_velocity', Float64, queue_size=10)
    rospy.init_node('forward_velocity', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #hello_str = "this is the angle %s" % rospy.get_time() % steering_angle
        rospy.loginfo(forward_velocity_command)
        pub.publish(forward_velocity_command)
        rate.sleep()

if __name__=='__main__':
    try:
        publish_forward_velocity(forward_velocity)
    except rospy.ROSInterruptException:
        pass
