#!/usr/bin/env python
# @author Kawin Nikomborirak
# @date 2018/04/10
# @summary node which takes destination coordinates and publishes
# steering angle and speed

import rospy
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import numpy as np
from gravl.msg import Hemisphere


class driveToPoint:
    def __init__(self, dest = np.array([42.294055, -71.264568])):
        rospy.init_node('driveToPoint')

        self.dest = dest

        self.turnrad = 2
        self.dep = np.array([0, 0])
        self.altitude = 0

        self.currentHeading = 0.0
        self.desiredHeading = 0.0
        self.pubAcker = rospy.Publisher(
            '/autodrive', AckermannDrive, queue_size=10)
        self.currentHeadingSub = rospy.Subscriber(
            '/heading', Hemisphere, self.currentHeadingCallback)
        self.desiredHeadingSub = rospy.Subscriber(
            '/course', Float32, self.desiredHeadingCallback)
        self.currentPositionSub = rospy.Subscriber(
            '/gps/fix', NavSatFix, self.positionCallback)

        self.ackMsg = AckermannDrive()
        self.ackMsg.speed = 1

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

    def desiredHeadingCallback(self, data):
        self.desiredHeading = data.data

    def currentHeadingCallback(self, data):
        print(data)
        self.currentHeading = data.direction

    def positionCallback(self, data):
        self.dep = np.array([data.latitude, data.longitude])
        self.altitude = data.altitude

    def publish(self):
        radius = 6371000 + self.altitude
        latToM = np.pi * radius / 180
        self.ackMsg.steering_angle = np.radians(
            self.currentHeading - self.desiredHeading)
        latDist = self.dep - self.dest
        mDist = latDist * latToM
        mDist[1] *= np.cos(np.radians(self.dest[0]))
        d = np.linalg.norm(mDist)

        self.ackMsg.speed = 1 if d > 1 else 0
        self.pubAcker.publish(self.ackMsg)


if __name__ == '__main__':
    driveToPoint()
