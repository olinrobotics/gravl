#!/usr/bin/env python

import rospy 
import math
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, Vector3, Point, Quaternion
from std_msgs.msg import ColorRGBA, Header
from ackermann_msgs.msg import AckermannDrive

class AckTest():

	def __init__(self):
		rospy.init_node('AckTest', anonymous=True)
		self.ackPub = rospy.Publisher("/cmd_vel",AckermannDrive,queue_size=10)
		self.update_rate = rospy.Rate(5)

	def run(self):
		while not rospy.is_shutdown():
			ack = AckermannDrive()
			for i in range(-70,70,1):
				ack.steering_angle = i / 100.0
				self.ackPub.publish(ack)
				self.update_rate.sleep()
			
		
if __name__ == '__main__':
    obs = AckTest()
    obs.run()