#!/usr/bin/env python
######################################################################
 # KUBO Safety Behavior Node (Teensy 3.5)
 # @file LidarCodeAdvanced.py
 # @author: Nathan Estill
 # @email: nathan.estill@students.olin.edu
 # @version: 2.0
 #
 # Sensing with the Lidar, detecting obstacles,
 # publishing True to estop if an obstacle is seen
 ######################################################################
import rospy 
import math
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
class ObstacleDetection():

    def __init__(self):
        # Variables explained in Github Wiki: https://github.com/olinrobotics/gravl/wiki/Vehicle-Safety
        rospy.init_node('Obstacle Detection', anonymous=True)
        self.hasSensed = False # If there is an obstacle sensed
        self.pubEstop = rospy.Publisher('/softestop', Bool,  queue_size=10) # Publisher that publishes to the estop
        self.laserSub = rospy.Subscriber('scan',LaserScan,self.callback) # Subscribes to the LIDAR, calls the callback function
        self.DistanceToTheGround = 4.5 # Essentially the ground
        self.widthTractor = 1.25 # Horizontal length of the tractor
        self.numberOfPointsNeededToTrigger = 15 # How many points must be seen to trigger a stop?
        self.update_rate = rospy.Rate(5)

    def lidarCB(self,data):
        # Collects data from the Hokuyo Lidar and stores it
        self.data = data

    def convertToVerticalAndHorizontal(self):
        self.totalDist = [] # Initializing arrays
        self.xDist = [] # Distance from front of tractor
        self.yDist = [] # Distance from center of tractor
        for i in range(len(self.data.ranges)): # Puts the tuple of data into x and y Distances
            self.totalDist.append(self.data.ranges[i])      
            self.xDist.append(abs(math.cos(math.radians((i - 380.0) * (190.0 / 760.0))) * self.totalDist[i])) # Computes the distance from the object
            self.yDist.append(math.sin(math.radians((i - 380.0) * (190.0 / 760.0))) * self.totalDist[i]) # Computes the distance parallel to tractor

    def getNumberOfObstacles(self):
        # Calculates the number of points that pose a threat to the tractor
        self.obstaclePoints = 0 # Counts how many points are not the ground
        self.triggerPoints = 0 # Counts number of points breaking threshold 
        for i in range(len(self.totalDist)): # Sweep through the distances
            if(self.totalDist[i] < self.DistanceToTheGround): # Is there an object that is not the ground?
                self.obstaclePoints += 1 # Add a point into the number of obstacle points
                if(abs(self.yDist[i]) < (self.widthTractor / 2.0)): #Will the obstacle hit the tractor?
                    self.triggerPoints += 1 # Add a point the the number of triggers

    def sendMessages(self):
        # If the number of trigger points is greater than the threshold, send a singal message to the tractor
        if(triggerPoints > numberOfPointsNeededToTrigger and not self.hasSensed): # if there is an obstacle that will hit the tractor
            # stop the tractor
            pub0.publish(True)
            self.hasSensed = True
        if(triggerPoints <= numberOfPointsNeededToTrigger and self.hasSensed):
            # don't stop the tractor
            pub0.publish(False)
            self.hasSensed = False

    def run(self):
        # Runs the code
        while not rospy.is_shutdown() and (self.data == None):
            self.convertToVerticalAndHorizontal()
            self.getNumberOfObstacles()
            self.sendMessages()
            self.update_rate.sleep()
if __name__ == '__main__':
    obs = ObstacleDetection()
    obs.run()