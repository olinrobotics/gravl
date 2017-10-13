#!/usr/bin/env python
import rospy 
from sensor_msgs.msg import LaserScan 
from std_msgs.msg import String 
from std_msgs.msg import Header 
import genpy
from std_msgs.msg import String

def callback(data):
    results = []
    i = 0
    while i < len(data.ranges):
        results.append(data.ranges[i])
        if results[i] > 1000000:
            results[i] = 100000
        i += 1
    rospy.loginfo(max(results))
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('scan',LaserScan,callback)
    rospy.spin()
def otherCode():
    totalDist = data.ranges # puts the distance away into a separate variable
    degrees = (step - 384) * (180 / 510) # Converts step to degrees
    for i in totalDist:
        horizontalDistance[i] = abs(sin((i - 384) * (180 / 512)) * totalDist) # Computes the distance parallel to tractor
        verticalDistance = abs(cos((i - 384) * (180 / 512)) * totalDist) # Computes the distance from the object
    someDistanceAway = 300 # Essentially the ground ***
    lengthOfTheTractor = 150 # Horizontal length of the tractor ***
    obstaclePoints = 0 # Counts how many points break the threshold
    numberOfPointsNeededToTrigger = 6 # How many points must be seen to trigger a stop? ***
    for i in totalDist: #Sweep throught the distances
        if(totalDist[i] < someDistanceAway and horizontalDistance[i] < 150): # Is there an object that will hit the tractor?
            obstaclePoints += 1 # Add a point into the number of obstacle points
    if(obstaclePoints > numberOfPointsNeededToTrigger):
        stopTheTractor # Whatever code is needed to stop the tractor

if __name__ == '__main__':
    listener()