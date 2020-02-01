#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Bool, Header, String, Float64, Float32
from sensor_msgs.msg import LaserScan
import genpy
import numpy as np
from geometry_msgs.msg import Point,PointStamped
class ObstacleDetector():
    def __init__(self):
        self.pub0 = rospy.Publisher('/estop', Bool,  queue_size=10) # init your publishers early
        self.pubObstPoint = rospy.Publisher('/point2follow', PointStamped, queue_size=10)
        rospy.init_node('ObstacleDetector', anonymous=True)
        self.subScan = rospy.Subscriber('/front/scan',LaserScan,self.callback)
        self.rate = rospy.Rate(1)
        self.scan = None

    def callback(self,data):
        self.scan = data

    def detectObstacles(self):
    # Code is supposed to detect if there is an obstacle, and if so, stop the tractor
    # lines with *** coordinated to the bravobot - 10/15/17
    # Things do to before completion:
    # TODO: Fix this up a little bit and make it more robust
    # Actually import data -- Done
    # Find out how to stop the tractor -- In Progress
    # Test various values to make sure it works
        totalDist = [] #Setting arrays
        verticalDistance = [] # Distance from front of tractor
        horizontalDistance = [] # Distance from center of tractor
        for i in range(len(self.scan.ranges)):
            totalDist.append(self.scan.ranges[i])
            if totalDist[i] > 1000000:
                totalDist[i] = 100000
            verticalDistance.append(abs(math.cos(math.radians((i - 380.0) * (190.0 / 760.0))) * totalDist[i])) # Computes the distance from the object
            horizontalDistance.append(math.sin(math.radians((i - 380.0) * (190.0 / 760.0))) * totalDist[i]) # Computes the distance parallel to tractor
        someDistanceAway = 4.5 # Essentially the ground ***
        lengthOfTheTractor = 1.2# Horizontal length of the tractor ***
        obstaclePoints = [] # Counts how many points are not the ground
        triggerPoints = 0 # Counts number of points breaking threshold
        numberOfPointsNeededToTrigger = 15 # How many points must be seen to trigger a stop? ***
        sumOfVert = [] #initializing the sum of the vertical points
        sumOfHor = [] #initializing the sum of the horizontal points
        obsNumber = -1#in case there are more than one obstacle
        obsCount = 20 #if there is significant distince between obs points, treat as diffferent obs
        for i in range(len(totalDist)): #Sweep through the distances
            if(totalDist[i] < someDistanceAway): # Is there an object that is not the ground?
                if(obsCount > 10):
                    obsNumber += 1
                    sumOfVert.append(0) # kind of calculate average Vertical distance eventually
                    sumOfHor.append(0) # kind of calculate average Horizontal distance eventually
                    obstaclePoints.append(0) # Add a point into the number of obstacle points
                sumOfVert[obsNumber] += verticalDistance[i] # kind of calculate average Vertical distance eventually
                sumOfHor[obsNumber] += horizontalDistance[i] # kind of calculate average Horizontal distance eventually
                obstaclePoints[obsNumber] += 1
                if(abs(horizontalDistance[i]) < (lengthOfTheTractor / 2.0)): #Will the obstacle hit the tractor?
                    triggerPoints += 1 # Add a point the the number of triggers
                obsCount = 0 # reset obsCount
            else:
                obsCount += 1
        sumOfVert = np.array(sumOfVert,dtype=np.float)
        sumOfHor = np.array(sumOfHor,dtype=np.float)
        newObsPts = np.array(obstaclePoints,dtype = np.float)
        for i in range(len(obstaclePoints)):
            if(((sumOfVert[i] / newObsPts[i]) < 0.02)):
                sumOfHor[i] = 35000
        absoSumOfHor = np.absolute(sumOfHor / newObsPts)
        targetObstacle = np.argmin(absoSumOfHor)
        point = Point()
        point.x = sumOfVert[targetObstacle] / obstaclePoints[targetObstacle] # Computes average distance of obstacle from tractor
        point.y = sumOfHor[targetObstacle] / obstaclePoints[targetObstacle] # Computes avearge distance from center of tractor
        point.z = 0
        pointStamped = PointStamped()
        pointStamped.point = point
        pointStamped.header = Header()
        pointStamped.header.frame_id = ('/hood')
        self.pubObstPoint.publish(pointStamped)

    def main(self):
        while not rospy.is_shutdown():
            if(self.scan != None):
                self.detectObstacles()
            self.rate.sleep()

if __name__ == '__main__':
    obstec = ObstacleDetector()
    obstec.main()
    rospy.spin()
