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

def callback(data):
    
    totalDist= []
    i = 0
    while i < len(data.ranges):
        totalDist.append(data.ranges[i])
        if totalDist[i] > 1000000:
            totalDist[i] = 100000 
        i += 1
    otherCode(data, 0)
    #print(totalDist[600])
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('scan',LaserScan,callback)
    rospy.spin()
def otherCode(data, angle):
    i = 0
    ang_res = 0.25
    # puts the distance away into a separate variable # Converts step to degrees
    totalDist = []
    verticalDistance = []
    horizontalDistance = []
    while i < len(data.ranges):
        totalDist.append(data.ranges[i])
        if totalDist[i] > 1000000:
            totalDist[i] = 100000        
        verticalDistance.append(abs(math.cos(math.radians(angle + (i - 380.0) * (190.0 / 760.0))) * totalDist[i])) # Computes the distance from the object
        horizontalDistance.append(math.sin(math.radians(angle + (i - 380.0) * (190.0 / 760.0))) * totalDist[i]) # Computes the distance parallel to tractor
        i += 1
    #horizontalDistance = [abs(sin((totalDist.index(i) - 384) * (180 / 512)) * i) for i in totalDist] 
    #verticalDistance = abs(cos((i - 384) * (180 / 512)) * totalDist) 
    someDistanceAway = 4.5 # Essentially the ground ***
    lengthOfTheTractor = 1.2# Horizontal length of the tractor ***
    obstaclePoints = 0 # Counts how many points break the threshold
    triggerPoints = 0
    numberOfPointsNeededToTrigger = 15 # How many points must be seen to trigger a stop? ***
    sumOfVert = 0
    sumOfHor = 0
    i = 15
    while i < len(totalDist)-15: #Sweep throught the distances
        if(totalDist[i] < someDistanceAway): # Is there an object that will hit the tractor?
            obstaclePoints += 1 # Add a point into the number of obstacle points
            sumOfVert += verticalDistance[i] # kind of calculate average Vertical distance eventually
            sumOfHor += horizontalDistance[i] # kind of calculate average Horizontal distance eventually
            if(abs(horizontalDistance[i]) < (lengthOfTheTractor / 2.0)):
                triggerPoints += 1
             
            #rospy.loginfo(i)
        i += 1
    averageVert = Float64()
    averageHor = Float64()
    averageNull = Float64()
    averageNull.data = -1
    if(triggerPoints > numberOfPointsNeededToTrigger):
        #stopTheTractor()    # Whatever code is needed to stop the tractor
        
        pub0 = rospy.Publisher('/estop', Bool,  queue_size=10)
        pub0.publish(True)
        if (abs(angle) == 0):
            otherCode(data,angle + 45)

    else:
        puback = rospy.Publisher('/myMadeUpTopicAck',Float32, queue_size=10)
        anglePub = Float32()
        anglePub.data = angle
        puback.publish(anglePub)
        pub0 = rospy.Publisher('/estop', Bool,  queue_size=10)
        pub0.publish(False)
    #elif(obstaclePoints > 0):
    #    averageVert.data = sumOfVert / obstaclePoints
    #    averageHor.data = sumOfHor / obstaclePoints    
    #    pub1 = rospy.Publisher('/scan_verticals', Float64,  queue_size=10)
    #   pub1.publish(averageVert)
    #    pub2 = rospy.Publisher('/scan_horizontals', Float64,  queue_size=10)
    #    pub2.publish(averageHor) 
    #    #rospy.loginfo(obstaclePoints)
    #else:
    #    pub1 = rospy.Publisher('/scan_verticals', Float64,  queue_size=10)
    #    pub1.publish(averageNull)
    #    pub2 = rospy.Publisher('/scan_horizontals', Float64,  queue_size=10)
    #    pub2.publish(averageNull)    #rospy.loginfo(totalDist[128])
    #rospy.loginfo(triggerPoints)
if __name__ == '__main__':
    listener()