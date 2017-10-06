#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose


class plotter:
    def __init__(self):
        self.i = 0
        self.f = open("toPlotRTK", 'w')
        rospy.init_node('converter')
        subscriber = rospy.Subscriber('/gps/fix', NavSatFix, self.plot)
        rospy.on_shutdown(self.close_file)

    def plot(self, msg):
        self.i+=1
        #if self.i%10 == 0:
        self.f.write('%.5f'%msg.latitude+","+'%.5f'%msg.longitude+"\n")

    def close_file(self):
        self.f.close()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    p = plotter()
    p.run()