#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix


class gpsToB:
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


    def callback(self, data):
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
