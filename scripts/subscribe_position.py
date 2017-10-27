#!/usr/bin/env python

#successfully gets data from bag, need to figure out how to actually use data in code

import rospy
from sensor_msgs.msg import NavSatFix


def callback(data):
    rospy.loginfo(data.latitude)
    rospy.loginfo(data.longitude)

def subscribe_position():
    rospy.init_node('fix', anonymous=True)
    rospy.Subscriber('fix', NavSatFix, callback)
    rospy.spin()
    print(x)

if __name__ == '__main__':
    subscribe_position()
