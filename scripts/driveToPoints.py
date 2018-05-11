#!/usr/bin/env python
# @author Kawin Nikomborirak
# @date 2018/04/10
# @summary node which takes destination coordinates and publishes
# steering angle and speed

import rospy
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import NavSatFix
import numpy as np
from gravl.msg import Hemisphere


class driveToPoint:
    def __init__(self):
        rospy.init_node('driveToPoint')

        # destination coordinates
        self.dests = np.array(rospy.get_param(
            "~points", [[42.294055, -71.264568]]))
        self.currentPoint = 0
        self.dest = self.dests[self.currentPoint]

        # turning radius of tractor [m]
        self.turnRad = 2
        self.inProgress = True

        # current location, to be changed via callback
        self.dep = np.array([0, 0])
        self.altitude = 0

        self.currentHeading = 0.0
        self.pubAcker = rospy.Publisher(
            '/autodrive', AckermannDrive, queue_size=10)
        self.currentHeadingSub = rospy.Subscriber(
            '/heading', Hemisphere, self.currentHeadingCallback)
        self.currentPositionSub = rospy.Subscriber(
            '/gps/fix', NavSatFix, self.positionCallback)

        self.ackMsg = AckermannDrive()
        self.ackMsg.speed = 1

        rate = rospy.Rate(10)

        while not rospy.is_shutdown() and self.inProgress:
            self.publish()
            rate.sleep()

    def currentHeadingCallback(self, data):
        self.currentHeading = data.direction

    def positionCallback(self, data):
        self.dep = np.array([data.latitude, data.longitude])
        self.altitude = data.altitude

    def getHeading(self):
        p1 = np.radians(self.dep)
        p2 = np.radians(self.dest)
        sina = np.cos(p2[0]) * np.sin(p2[1] - p1[1])
        cosa = np.cos(p1[0]) * np.sin(p2[0]) - np.sin(p1[0]) * \
            np.cos(p2[0]) * np.cos(p2[1] - p1[1])
        angle = np.arctan2(sina, cosa)
        angle += 2 * np.pi if angle < 0 else 0
        self.desiredHeading = np.degrees(angle)

    def publish(self):
        # update desired heading
        self.getHeading()

        radius = 6371000 + self.altitude

        # meters per change in latitude
        latToM = np.pi * radius / 180

        self.ackMsg.steering_angle = np.radians(
            self.currentHeading - self.desiredHeading)

        # put the angles in -pi to pi domain
        if self.ackMsg.steering_angle > np.pi:
            self.ackMsg.steering_angle -= 2 * np.pi
        if self.ackMsg.steering_angle < -np.pi:
            self.ackMsg.steering_angle += 2 * np.pi

        latDist = self.dep - self.dest
        mDist = latDist * latToM
        mDist[1] *= np.cos(np.radians(self.dest[0]))

        # distance from target
        d = np.linalg.norm(mDist)

        # This commented block is to teardrop turn in case the point is too close.
        # This is untested, so it is commented out.
        # # minimum distance a point can be to make a direct turn
        # minD = 2 * self.turnRad * \
        #     np.sin(np.radians(self.desiredHeading -
        #                       self.currentHeadinsibg)) * 2 * np.pi

        # # if the point is to close, teardrop turn.
        # if d < minD:
        #     self.ackMsg.steering_angle = (
        #         (self.currentHeading - self.desiredHeading < 0) * 2 - 1) * np.pi / 4

        # self.ackMsg.speed = 1 if d > 1 else 0

        if d < 1:
            if self.currentPoint == self.dests.shape[0] - 1:
                self.ackMsg.speed = 0
                self.inProgress = False
            else:
                self.currentPoint += 1
                self.dest = self.dests[self.currentPoint]

        self.pubAcker.publish(self.ackMsg)


if __name__ == '__main__':
    driveToPoint()
