#!/usr/bin/env python

import rospy 
import math
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
class ObstacleDetection():

    def __init__(self):
        rospy.init_node('Obstacle Detection', anonymous=True)
        self.hasSensed = False
        self.pubEstop = rospy.Publisher('/softestop', Bool,  queue_size=10)
        self.laserSub = rospy.Subscriber('scan',LaserScan,self.callback) # Subscribes to the LIDAR, calls the callback function
        self.DistanceToTheGround = 4.5 # Essentially the ground ***
        self.lengthOfTheTractor = 1.25 # Horizontal length of the tractor ***
        self.numberOfPointsNeededToTrigger = 15 # How many points must be seen to trigger a stop? *** 

    def callback(self,data):
        self.data = data #calling below code

    def convertToVerticalAndHorizontal(self):
        self.totalDist = [] # Initializing arrays
        self.verticalDistance = [] # Distance from front of tractor
        self.horizontalDistance = [] # Distance from center of tractor
        for i in range(len(self.data.ranges)):
            self.totalDist.append(self.data.ranges[i])
            if self.totalDist[i] > 1000000:
                self.totalDist[i] = 100000        
            self.verticalDistance.append(abs(math.cos(math.radians((i - 380.0) * (190.0 / 760.0))) * self.totalDist[i])) # Computes the distance from the object
            self.horizontalDistance.append(math.sin(math.radians((i - 380.0) * (190.0 / 760.0))) * self.totalDist[i]) # Computes the distance parallel to tractor

    def getNumberOfObstacles(self):
        self.obstaclePoints = 0 # Counts how many points are not the ground
        self.triggerPoints = 0 # Counts number of points breaking threshold 
        for i in range(len(self.totalDist)): # Sweep through the distances
            if(self.totalDist[i] < self.DistanceToTheGround): # Is there an object that is not the ground?
                self.obstaclePoints += 1 # Add a point into the number of obstacle points
                if(abs(self.horizontalDistance[i]) < (self.lengthOfTheTractor / 2.0)): #Will the obstacle hit the tractor?
                    self.triggerPoints += 1 # Add a point the the number of triggers

    def sendMessages(self):
    # Code is supposed to detect if there is an obstacle, and if so, stop the tractor
    # Rougly complete, may not work in parcel B
        if(triggerPoints > numberOfPointsNeededToTrigger and not self.hasSensed): # if there is an obstacle that will hit the tractor
            # stop the tractor
            pub0.publish(True)
            self.hasSensed = True
        if(triggerPoints <= numberOfPointsNeededToTrigger and self.hasSensed):
            # don't stop the tractor
            pub0.publish(False)
            self.hasSensed = False

    def run(self):
        self.convertToVerticalAndHorizontal()
        self.getNumberOfObstacles()
        self.sendMessages()

if __name__ == '__main__':
    obs = ObstacleDetection()
    obs.listener()
    rospy.spin()