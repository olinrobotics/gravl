#!/usr/bin/env python

import rospy
from turnToHeading import turnHeading
from gpsToB import gpsToB
from sensor_msgs.msg import NavSatFix


class goToPoints:
    """Supply a list of (latitude, longitude) tuples in the
    constructor and make the tractor go to each point"""

    def __init__(self, locations):
        rospy.init_node('gpsToPoints')

        self.locations = locations

        turnHeading()

        self.lat = 0
        self.lon = 0
        self.loc_sub = rospy.Subscriber('/gps/fix', NavSatFix, self.callback)
        dest = gpsToB(locations[0][0], locations[0][1])

        for location in locations:
            dest.setDestination(location[0], location[1])
            while True:
                # while still more than a meter from destination
                if ((location[0] - self.lat) ** 2 + (location[1] - self.lon) ** 2) ** .5 < 1.0 / 60 / 1852 / .04:
                    break

        # keep tractor in the final destination
        dest.setDestination(self.lat, self.lon)

    def callback(self, data):
        self.lat = data.latitude
        self.lon = data.longitude
