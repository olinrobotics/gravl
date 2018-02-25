#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix


class gpsToB:
    '''Starts a node which takes the destination latitude and
    longitude in the constructor and publishes the bearing to the
    destination.'''
    def __init__(self, destLat, destLong):
        rospy.init_node('gpsToB')
        self.course_pub = rospy.Publisher('course', Float32, queue_size=10)
        self.loc_sub = rospy.Subscriber('/gps/fix', NavSatFix, self.callback)
        self.destLat = destLat
        self.destLong = destLong
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print('Shutting down')

    def setDestination(self, destLat, destLong):
        '''Set the destination longitude and latitude'''
        self.destLat = destLat
        self.destLong = destLong


    def callback(self, data):
        '''Publish the bearing. pdf here:
        https://badarabbas.files.wordpress.com/2011/03/paper.pdf'''
        lat1 = np.radians(data.latitude)
        lon1 = np.radians(data.longitude)
        lat2 = np.radians(self.destLat)
        lon2 = np.radians(self.destLong)
        sina = np.cos(lat2) * np.sin(lon2 - lon1)
        cosa = np.cos(lat1) * np.sin(lat2) - np.sin(lat1) * np.cos(lat2) * np.cos(lon2 - lon1)
        angle = np.arctan2(sina, cosa)
        self.course_pub.publish(np.degrees(angle))


if __name__ == '__main__':
    gpsToB(72.29, -71.26)
