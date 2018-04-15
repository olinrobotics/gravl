#!/usr/bin/env python
# @author Kawin Nikomborirak
# @date 2018/02/25
# summary node which takes destination coordinates and publishes
# bearing.


import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix


class gpsToB:
    '''Starts a node which takes the destination latitude and
    longitude in the constructor and publishes the bearing to the
    destination.'''

    def __init__(self, dest):
        rospy.init_node('gpsToB')
        self.course_pub = rospy.Publisher('course', Float32, queue_size=10)
        self.loc_sub = rospy.Subscriber('/gps/fix', NavSatFix, self.callback)
        self.dest = dest
        self.dep = np.array([0,0])
        self.depSubscriber = rospy.Subscriber(
            '/gps/fix', NavSatFix, self.callback)

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

    def setDestination(self, destLat, destLng):
        '''Set the destination longitude and latitude'''
        self.dep = np.array([destLat, destLng])

    def callback(self, data):
        '''Publish the bearing. pdf here:
        https://badarabbas.files.wordpress.com/2011/03/paper.pdf'''
        self.dep = np.array([data.latitude, data.longitude])

    def publish(self):
        p1 = np.radians(self.dep)
        p2 = np.radians(self.dest)
        sina = np.cos(p2[0]) * np.sin(p2[1] - p1[1])
        cosa = np.cos(p1[0]) * np.sin(p2[0]) - np.sin(p1[0]) * \
            np.cos(p2[0]) * np.cos(p2[1] - p1[1])
        angle = np.arctan2(sina, cosa)
        self.course_pub.publish(np.degrees(angle))


if __name__ == '__main__':
    gpsToB(np.array([42.294055, -71.264568]))
