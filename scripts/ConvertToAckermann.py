#!/usr/bin/env python
"""
Subscribes to a twist message and converts it to an ackermann message.
Subscribes to /cmd_twist
Publishes to /cmd_vel

Assumes all inputs are normalized between -1 and 1

@edited: 12/02/2018
@author: Amy Phung
"""

import rospy
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive
from scipy.interpolate import interp1d

class ConvertToAckermann():
    def __init__(self):
        # Define variables
        self.twist_data = None

        # Define ROS constructs
        rospy.init_node("convert_to_ackermann")
        self.twist_sub = rospy.Subscriber("/cmd_twist", Twist, self.twist_cb)
        self.ack_pub = rospy.Publisher("/cmd_vel", AckermannDrive, queue_size=1)
        self.update_rate = rospy.Rate(10)

    def twist_cb(self,msg):
        self.twist_data = msg

    def twist_to_ackermann(self,linear_vel, angular_vel):
        """
        Converts linear and angular velocities to linear velocity and steering angle for
        ackermann messages

        Args:
            linear_vel - forward linear velocity from Twist message (should be between -1 and 1)
            angular_vel - angular velocity from Twist message (should be between -1 and 1)
        """
        # Assume twist message is angular vel from -1 to 1, velocity is -1 to 1
        # steering in degrees from -45 to 45, velocity is -1 to 1
        vel_scale = interp1d([-1,1],[-1,1])
        angle_scale = interp1d([-1,1],[-45,45])

        ack_msg = AckermannDrive()
        ack_msg.speed = vel_scale(linear_vel)
        ack_msg.steering_angle = angle_scale(angular_vel)

        return ack_msg

    def run(self):
        # Takes no args, executes timed loop for node
        while not rospy.is_shutdown():
            if self.twist_data == None:
                rospy.loginfo('MSG: No twist data published')
                self.update_rate.sleep()
                continue
            linear = self.twist_data.linear.x
            angular = self.twist_data.angular.z

            ack_msg = self.twist_to_ackermann(linear, angular)
            self.ack_pub.publish(ack_msg)
            self.update_rate.sleep()

if __name__ == "__main__":
    convert_to_ackermann = ConvertToAckermann()
    convert_to_ackermann.run()
