#!/usr/bin/env python

import sys
import time
import rospy
from sensor_msgs.msg import Imu                 # ROS msg type for IMU
from Phidget22.Devices.Gyroscope import *
from Phidget22.Devices.Accelerometer import *
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Net import *

rospy.init_node('imu')
pub = rospy.Publisher('imu_data', Imu, queue_size = 10)

if __name__ == __main__:

    # Setting up Phidget object
    try:
        ch = Gyroscope()
    except RuntimeError as e:
        print("Runtime Exception %s" % e.details)
        print("Press Enter to Exit...\n")
        readin = sys.stdin.read(1)
        exit(1)
    try:
        ch.open()
    except PhidgetException as e:
        print (“Phidget Exception %i: %s” % (e.code, e.details))

    # Opening ROS publisher
    rospy.init('imu_data', anonymous=True)
    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        pub.publish()
        r.sleep()
