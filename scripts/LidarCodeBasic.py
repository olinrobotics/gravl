#!/usr/bin/env python

import rospy 
import math
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan 
from std_msgs.msg import String 
from std_msgs.msg import Header 
import genpy
from std_msgs.msg import String
from std_msgs.msg import Float64
class ObstacleDetection():
    def __init__(self):
        self.hasSensed = False
    def callback(self,data):
        totalDist= [] #Make a new array, this stuff is currently just for debugging, and unneccesary
        i = 0 # Using a manual for loop because i dont know python
        while i < len(data.ranges): # for loop stuff
            totalDist.append(data.ranges[i]) # assign each range to the array to make it canfigurable
            if totalDist[i] > 1000000: # converting all 'inf' to an actual number
                totalDist[i] = 100000 
            i += 1 # parsing
        self.otherCode(data) #calling below code
        #print(totalDist[600])
    def listener(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('listener', anonymous=True) # idk what this does tbh
        rospy.Subscriber('scan',LaserScan,self.callback) # Subscribes to the laser thing, calls the callback function
    def otherCode(self,data):
    # Code is supposed to detect if there is an obstacle, and if so, stop the tractor
    # Rougly complete, may not work in parcel B
        pub0 = rospy.Publisher('/softestop', Bool,  queue_size=10) # init your publishers early
        pub1 = rospy.Publisher('/scan_verticals', Float64,  queue_size=10)
        pub2 = rospy.Publisher('/scan_horizontals', Float64,  queue_size=10) 
        totalDist = [] #Setting arrays
        verticalDistance = [] # Distance from front of tractor
        horizontalDistance = [] # Distance from center of tractor
        i = 0
        while i < len(data.ranges):
            totalDist.append(data.ranges[i])
            if totalDist[i] > 1000000:
                totalDist[i] = 100000        
            verticalDistance.append(abs(math.cos(math.radians((i - 380.0) * (190.0 / 760.0))) * totalDist[i])) # Computes the distance from the object
            horizontalDistance.append(math.sin(math.radians((i - 380.0) * (190.0 / 760.0))) * totalDist[i]) # Computes the distance parallel to tractor
            i += 1
        someDistanceAway = 4.5 # Essentially the ground ***
        lengthOfTheTractor = 1.2# Horizontal length of the tractor ***
        obstaclePoints = 0 # Counts how many points are not the ground
        triggerPoints = 0 # Counts number of points breaking threshold
        numberOfPointsNeededToTrigger = 15 # How many points must be seen to trigger a stop? ***
        sumOfVert = 0 #initializing the sum of the vertical points
        sumOfHor = 0#initializing the sum of the horizontal points
        i = 0
        while i < len(totalDist): #Sweep throught the distances
            if(totalDist[i] < someDistanceAway): # Is there an object that is not the ground?
                obstaclePoints += 1 # Add a point into the number of obstacle points
                sumOfVert += verticalDistance[i] # kind of calculate average Vertical distance eventually
                sumOfHor += horizontalDistance[i] # kind of calculate average Horizontal distance eventually
                if(abs(horizontalDistance[i]) < (lengthOfTheTractor / 2.0)): #Will the obstacle hit the tractor?
                    triggerPoints += 1 # Add a point the the number of triggers
            i += 1
        averageVert = Float64() #average vertical distance of the obstacle
        averageHor = Float64() #average horizontal distance of the obstacle
        averageNull = Float64() # if there aren't any obstacles
        averageNull.data = -1
        if(triggerPoints > numberOfPointsNeededToTrigger and not self.hasSensed): # if there is an obstacle that will hit the tractor
            #  stop the tractor
            pub0.publish(True)
            self.hasSensed = True
        if(triggerPoints <= numberOfPointsNeededToTrigger):
            # don't stop the tractor
            self.hasSensed = False
        if(obstaclePoints > 0):
            averageVert.data = sumOfVert / obstaclePoints # Computes average distance of obstacle from tractor
            averageHor.data = sumOfHor / obstaclePoints # Computes avearge distance from center of tractor
            pub1.publish(averageVert) #Publishes
            pub2.publish(averageHor) #Pubslishes
        else:
            pub1.publish(averageNull) # -1 because no obstacles   
            pub2.publish(averageNull) # -1 because no obstacles

if __name__ == '__main__':
    obs = ObstacleDetection()
    obs.listener()
    rospy.spin()