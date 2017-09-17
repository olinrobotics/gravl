#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

def process_odom(msg):
    publisher.publish(msg.pose.pose)

rospy.init_node('converter')

publisher = rospy.Publisher('/tractor_position', Pose, queue_size=10)
subscriber = rospy.Subscriber('/gps/rtkfix', Odometry, process_odom)

rospy.spin()