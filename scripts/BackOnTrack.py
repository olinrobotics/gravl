#!/usr/bin/env python
import rospy 
import math
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan 
from std_msgs.msg import String 
from std_msgs.msg import Header 
import genpy
from std_msgs.msg import String
from std_msgs.msg import Float64, Float32
#import statistics
from numpy import median
from ackermann_msgs.msg import AckermannDrive
class TrackFollower():
    def callback(self,data):
        totalDist= [] #Make a new array, this stuff is currently just for debugging, and unneccesary
        i = 0 # Using a manual for loop because i dont know python
        while i < len(data.ranges): # for loop stuff
            totalDist.append(data.ranges[i]) # assign each range to the array to make it canfigurable
            if totalDist[i] > 1000000: # converting all 'inf' to an actual number
                totalDist[i] = 100000 
            i += 1 # parsing
        self.otherCode(data) #calling below code
        #print(totalDist[360])
    def listener(self,):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('listener', anonymous=True) # idk what this does tbh
        rospy.Subscriber('scan',LaserScan,self.callback) # Subscribes to the laser thing, calls the callback function
    def otherCode(self,data):
        pubAcker = rospy.Publisher('/autodrive', AckermannDrive, queue_size=10) # init publisher
        ack_msg = AckermannDrive() # initialize ackermann message
        i = 0
        speed = 1 #setting the speed
        totalDist = [] #Setting arrays
        angs = [] #for the angles that are road
        while i < len(data.ranges):
            totalDist.append(data.ranges[i])
            if totalDist[i] > 1000000:
                totalDist[i] = 100000       
            if totalDist[i] > 5 and totalDist[i] < 8: # is the point in the road?
                angs.append(math.radians((i - 380.0) * (190.0 / 760.0))) #log the angle (radians) 
            i += 1
        ack_msg.speed = speed # set speed to the ackermann message
        ack_msg.steering_angle = median(angs) # set the angle to the ackermann message
        pubAcker.publish(ack_msg)
        rospy.loginfo(median(angs))
if __name__ == '__main__':
    track = TrackFollower()
    track.listener()
    rospy.spin()