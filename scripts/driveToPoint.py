#!/usr/bin/env python
# @author Kawin Nikomborirak
# @date 2018/02/25
# @summary node which takes destination coordinates and publishes
# bearing.

import rospy
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32
from std_msgs.msg import Float64
import numpy as np


class turnHeading:
    def __init__(self, lon=0, lat=0):
        rospy.init_node('turnHeading')

        self.destLon = lon
        self.destLat = lat

        # turn radius in m.
        self.turnRad = 2

        self.pubAcker = rospy.Publisher(
            '/autodrive', AckermannDrive, queue_size=10)
        self.desiredHeadingSub = rospy.Subscriber(
            '/course', Float32, self.desiredHeadingCallback)
        self.currentHeadingSub = rospy.Subscriber(
            '/heading', Float64, self.currentHeadingCallback)
        self.currentPointSub = rospy.Subscriber(
            '/gps/fix', NavSatFix, self.positionCallback)

        self.ackMsg = AckermannDrive()
        self.ackMsg.speed = 1

        self.desiredHeading = 0
        self.currentHeading = 0

        try:
            rospy.spin()
        except KeyboardInterrupt:
            print('Shutting down')

    def desiredHeadingCallback(self, data):

        self.desiredHeading = data

    def currentHeadingCallback(self, data):
        self.currentHeading = data

    def positionCallBack(self, data):
        self.depLon = data.longitude
        self.depLat = data.latitude

        self.ackMsg.steering_angle = np.radians(
            self.currentHeading - self.desiredHeading % 360)

        lonDelta = (self.depLon - self.destLon) * 60 * 1852
        latDelta = (self.depLat - self.destLat) * 60 * 1852

        d = (lonDelta ** 2 + latDelta ** 2) ** .5
        maxD = np.sin(np.radians(self.desiredHeading -
                                 self.currentHeading)) * 2 * np.pi
        if d < maxD:
            self.ackMsg.steering_angle = ((self.currentHeading - self.desiredHeading < 0) * 2 - 1) * np.pi / 4


if __name__ == '__main__':
    turnHeading()
