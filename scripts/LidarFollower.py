#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_msgs.msg import Header
import genpy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64, Float32
from ackermann_msgs.msg import AckermannDrive
class ObstacleFollower():
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
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('scan',LaserScan,self.callback)
    def otherCode(self,data):
    # Code is supposed to detect if there is an obstacle, and if so, stop the tractor
    # lines with *** coordinated to the bravobot - 10/15/17
    # Things do to before completion:
    # Actually import data -- Done
    # Find out how to stop the tractor -- In Progress
    # Test various values to make sure it works
        pub0 = rospy.Publisher('/estop', Bool,  queue_size=10) # init your publishers early
        pubAcker = rospy.Publisher('/autodrive', AckermannDrive, queue_size=10)
        ack_msg = AckermannDrive()
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
        obstaclePoints = [] # Counts how many points are not the ground
        triggerPoints = 0 # Counts number of points breaking threshold
        numberOfPointsNeededToTrigger = 15 # How many points must be seen to trigger a stop? ***
        sumOfVert = [] #initializing the sum of the vertical points
        sumOfHor = [] #initializing the sum of the horizontal points
        obsNumber = -1#in case there are more than one obstacle
        obsCount = 20 #if there is significant distince between obs points, treat as diffferent obs
        i = 0
        while i < len(totalDist): #Sweep throught the distances
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
            i += 1
        i = 0
        while i < len(obstaclePoints):
            if(obstaclePoints[i] < 16):
                sumOfHor[i] = 3500
            i += 1
        newSumOfVert = np.array(sumOfVert,dtype=np.float)
        newSumOfHor = np.array(sumOfHor,dtype=np.float)
        newObsPts = np.array(obstaclePoints,dtype = np.float)
        print("verticalData:",newSumOfVert / newObsPts)
        print("horizontalData",newSumOfHor / newObsPts)
        print("obstaclePoints = ",obstaclePoints)
        absoSumOfHor = np.absolute(newSumOfHor / newObsPts)
        #absoSumOfHor = absoSumOfHor.tolist()
        targetObstacle = np.argmin(absoSumOfHor)
        print(targetObstacle)
        averageVert = Float64() #average vertical distance of the obstacle
        averageHor = Float64() #average horizontal distance of the obstacle
        averageNull = Float64() # if there aren't any obstacles
        averageNull.data = -1
        averageVert.data = sumOfVert[targetObstacle] / obstaclePoints[targetObstacle] # Computes average distance of obstacle from tractor
        averageHor.data = sumOfHor[targetObstacle] / obstaclePoints[targetObstacle] # Computes avearge distance from center of tractor
        angle = math.tan(averageHor.data / averageVert.data) # Finds the angle at which the tractor should turn
        if (averageVert.data > 1): #If obstacle is far away, go fast
            speed = 0.2 + 0.5 * (averageVert.data) - 0.5
        else: # if obstacle is really close, stop moving
            speed = 0
        print("speed = ",speed)
        ack_msg.speed = speed # set speed to the ackermann message
        ack_msg.steering_angle = math.degrees(angle) # set the angle to the ackermann message
        if(abs(ack_msg.steering_angle) > 45 or averageVert.data < 1): #something about being too far away makes it turn really far, so i kinda hard coded it
            ack_msg.steering_angle = 0.0
        pubAcker.publish(ack_msg) # publish

if __name__ == '__main__':
    obstec = ObstacleFollower()
    obstec.listener()
    rospy.spin()
