#!/usr/bin/env python

#gets data from publish_velocity.py publisher
import rospy
from std_msgs.msg import Float64

def callback(data):
    rospy.loginfo(data.data)

def subscribe_position():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('vel', anonymous=True)

    rospy.Subscriber('vel', Float64, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    subscribe_position()
