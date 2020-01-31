#!/usr/bin/env python2
import rospy
import math
from geometry_msgs.msg import Point, Twist
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import String

class Follower():
    def __init__(self):
        rospy.init_node('Follower', anonymous=True)
        self.subPoint = rospy.Subscriber('/point2follow', Point, self.callback)
        self.pubDrive = rospy.Publisher('/state_controller/cmd_behavior_twist',Twist, queue_size=10)
        self.point = None
        self.rate = rospy.Rate(1)

    def callback(self,data):
        self.point = data

    def follow(self):
        drive_msg = Twist()
        drive_msg.label = String()
        drive_msg.label.data = "2D Follower"
        if (self.point.y > 1): #If obstacle is far away, go fast
            speed = 0.25 * (self.point.y - 1) 
        else: # if obstacle is really close, stop moving
            speed = 0
        if(speed > 2):
            speed = 2
        drive_msg.linear.x = speed
        angle = math.tan(self.point.y / self.point.x) # Finds the angle at which the tractor should turn
        angle = math.degrees(angle) # set the angle to the ackermann message
        if(angle > 45):
            angle = 45
        elif(angle < -45):
            angle = -45
        elif(self.point.y < 1): 
            angle = 0.
        drive_msg.angular.z = angle / 45
        self.pubDrive.publish(drive_msg) # publish

    def main(self):
        while not rospy.is_shutdown():
            if(self.point != None):
                self.follow()
            self.rate.sleep()

if __name__ == '__main__':
    follower = Follower()
    follower.main()