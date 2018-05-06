#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np


class RtkToPoints:
    def __init__(self, dests=np.reshape([10, 10], (1, 2))):
        rospy.init_node('rtk_to_points')

        # destination coordinates
        self.dests = dests
        self.currentPoint = 0

        # current destination
        self.dest = self.dests[self.currentPoint]

        # turning radius of tractor [m]
        self.turnRad = 2

        # default current location and heading
        self.dep = np.array([0, 0])
        self.currentHeading = 0.0
        self.inProgress = True
        self.rate = rospy.Rate(10)

        self.ackMsg = AckermannDrive()
        self.ackMsg.speed = 1

        self.pubAcker = rospy.Publisher(
            '/autodrive', AckermannDrive, queue_size=10)

        self.subRtk = rospy.Subscriber(
            '/gps/rtkfix', Odometry, self.rtkCallback)

        def rtkCallback(self, data):
            pose = data.pose.pose
            self.currentHeading = euler_from_quaternion(
                [pose.orientation.x, pose.orientation.y,
                 pose.orientation.z, pose.orientation.w])[-1]
            self.dep = np.array(pose.position)[:-1]

        def spin(self):

            while not rospy.is_shutdown() and self.inProgress:
                vec = self.dest - self.dep
                theta = np.arctan2(vec[1], vec[0])
                self.ackMsg.steering_angle = theta

                d = np.linalg.norm(vec)

                # # minimum distance a point can be to make a direct turn
                # minD = 2 * self.turnRad * \
                #     np.sin(np.radians(theta)) * 2 * np.pi

                # # if the point is to close, teardrop turn.
                # if d < minD:
                #     self.ackMsg.steering_angle = (
                #         (self.currentHeading - self.desiredHeading < 0) * 2 - 1) * np.pi / 4

                if d < .1:
                    if self.currentPoint == self.dests.shape[0] - 1:
                        self.ackMsg.speed = 0
                        self.inProgress = False
                    else:
                        self.currentPoint += 1
                        self.dest = self.dests[self.currentPoint]

                self.pubAcker.publish(self.ackMsg)
                self.rate.sleep()



if __name__ == '__main__':
    rtkToPoints = RtkToPoints()
    rtkToPoints.spin()
