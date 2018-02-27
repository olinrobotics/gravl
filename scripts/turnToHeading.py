#!/usr/bin/env python
# @author Kawin Nikomborirak
# @date 2018/02/25
# @summary node which takes destination coordinates and publishes
# bearing.

import rospy
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32
from std_msgs.msg import Float64


class turnHeading:
    def __init__(self):
        rospy.init_node('turnHeading')

        self.pubAcker = rospy.Publisher(
            '/autodrive', AckermannDrive, queue_size=10)
        self.desiredHeadingSub = rospy.Subscriber(
            '/course', Float32, self.desiredHeadingCallback)
        self.currentHeadingSub = rospy.Subscriber(
            '/heading', Float64, self.currentHeadingCallback)
        self.ackMsg = AckermannDrive()
        self.desiredHeading = 0
        self.currentHeading = 0
        self.ackMsg.steering_angle = np.radians(self.currentHeading - self.desiredHeading)
        self.ackMsg.steering_angle = (
            (0 - self.desiredHeading > 0) * 2 - 1) * 45
        self.ackMsg.speed = 1
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print('Shutting down')

    def desiredHeadingCallback(self, data):
        self.desiredHeading = data
        self.pubAcker.publish(self.ackMsg)

    def currentHeadingCallback(self, data):
        self.currentHeading = data


# if __name__ == '__main__':
#     turnHeading()
