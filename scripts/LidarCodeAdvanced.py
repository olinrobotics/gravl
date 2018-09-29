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
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, Vector3, Point, Quaternion
from std_msgs.msg import ColorRGBA, Header
class ObstacleDetection():

    def __init__(self):
        # Variables explained in Github Wiki: https://github.com/olinrobotics/gravl/wiki/Vehicle-Safety
        rospy.init_node('ObstacleDetection', anonymous=True)
        self.hasSensedFront = False # If there is an obstacle sensed
        self.hasSensedDown = False
        self.pubEstop = rospy.Publisher('/softestop', Bool,  queue_size=10) # Publisher that publishes to the estop
        self.frontlaserSub = rospy.Subscriber('/front/scan',LaserScan,self.frontLidarCB) # Subscribes to the front LIDAR, calls the callback function
        self.downLaserSub = rospy.Subscriber('/down/scan',LaserScan,self.downLidarCB) # Subscribes to the down Lidar, calls the cb function
        self.frontData = None   
        self.downData = None
        self.DistanceToTheGround = 4.5 # Essentially the ground
        self.DownToGround = 1.2 # Distance from down lidar to the ground
        self.widthTractor = 1.25 # Horizontal length of the tractor
        self.threshold = 15 # How many points must be seen to trigger a stop?
        self.update_rate = rospy.Rate(5)
        self.vis_pub = rospy.Publisher('/TractorLine_pt', Marker, queue_size=2)

    def frontLidarCB(self,data):
        # Collects data from the Front Hokuyo Lidar and stores it
        self.frontData = data

    def downLidarCB(self,data):
        # Collects data from the Down Hokuyo Lidar and stores it
        self.downData = data

    def convertToXDistAndYDistFront(self):
        self.totalDistFront = [] # Initializing arrays
        self.xDistFront = [] # Distance from front of tractor
        self.yDistFront = [] # Distance from center of tractor
        dataPoints = len(self.frontData.ranges)
        angleSweep = 190.0
        for i in range(len(self.frontData.ranges)): # Puts the tuple of data into x and y Distances
            self.totalDistFront.append(self.frontData.ranges[i])
            angleRad = math.radians((i - dataPoints / 2) * (angleSweep / dataPoints))      
            self.xDistFront.append(abs(math.cos(angleRad) * self.totalDistFront[i])) # Computes the distance from the object
            self.yDistFront.append(math.sin(angleRad) * self.totalDistFront[i]) # Computes the distance parallel to tractor

    def getNumberOfObstaclesFront(self):
        # Calculates the number of points that pose a threat to the tractor
        self.obstaclePointsFront = 0 # Counts how many points are not the ground
        self.triggerPointsFront = 0 # Counts number of points breaking threshold 
        for i in range(len(self.totalDist)): # Sweep through the distances
            if(self.totalDistFront[i] < self.DistanceToTheGround): # Is there an object that is not the ground?
                self.obstaclePointsFront += 1 # Add a point into the number of obstacle points
                if(abs(self.yDistFront[i]) < (self.widthTractor / 2.0)): #Will the obstacle hit the tractor?
                    self.triggerPointsFront += 1 # Add a point the the number of triggers

    def sendMessagesFront(self):
        # If the number of trigger points is greater than the threshold, send a singal message to the tractor
        if(self.triggerPointsFront > self.threshold and not self.hasSensedFront): # if there is an obstacle that will hit the tractor
            # stop the tractor
            self.pubEstop.publish(True)
            self.hasSensedFront = True
        if(self.triggerPointsFront <= self.threshold and self.hasSensedFront):
            # don't stop the tractor
            self.pubEstop.publish(False)
            self.hasSensedFront = False

    def convertToYDistAndZDistDown(self):
        self.totalDistDown = [] # Initializing arrays
        self.zDistDown = [] # Distance from front of tractor
        self.yDistDown = [] # Distance from center of tractor
        dataPoints = len(self.downData.ranges)
        angleSweep = 270.0
        for i in range(len(self.downData.ranges)): # Puts the tuple of data into x and y Distances
            self.totalDistDown.append(self.downData.ranges[i])
            angleRad = math.radians((i - dataPoints / 2) * (angleSweep / dataPoints))
            self.zDistDown.append(abs(math.cos(angleRad) * self.totalDistDown[i])) # Computes the distance from the object
            self.yDistDown.append(abs(math.sin(angleRad) * self.totalDistDown[i])) # Computes the distance parallel to tractor

    def getNumberOfObstaclesDown(self):
        # Calculates the number of points that pose a threat to the tractor
        self.obstaclePointsDown = 0 # Counts how many points are not the ground
        self.triggerPointsDown = 0 # Counts number of points breaking threshold
        for i in range(len(self.totalDistDown)): # Sweep through the distances
            if(self.zDistDown[i] < self.DownToGround and self.zDistDown[i] > 0.1): # Is there an object that is not the ground?
                self.obstaclePointsDown += 1 # Add a point into the number of obstacle points
                if(abs(self.yDistDown[i]) < (self.widthTractor / 2.0)): #Will the obstacle hit the tractor?
                    self.triggerPointsDown += 1 # Add a point the the number of triggers
        used_pts = Marker() # Marker to visualize used lidar pts
        used_pts.header.frame_id = "laser"
        used_pts.header.stamp = rospy.Time.now()
        used_pts.type = 8
        used_pts.scale = Vector3(100,0.01,0.01)
        used_pts.color = ColorRGBA(1,1,1,1)
        used_pts.points = [Point(0,1.25 / 2,0),Point(0,-1.25 / 2,0)]
        used_pts.pose.orientation.w = 1.0
        self.vis_pub.publish(used_pts)



    def sendMessagesDown(self):
        # If the number of trigger points is greater than the threshold, send a singal message to the tractor
        if(self.triggerPointsDown > self.threshold and not self.hasSensedDown): # if there is an obstacle that will hit the tractor
            # stop the tractor
            self.pubEstop.publish(True)
            self.hasSensedDown = True
        if(self.triggerPointsDown <= self.threshold and self.hasSensedDown):
            # don't stop the tractor
            self.pubEstop.publish(False)
            self.hasSensedDown = False
    def run(self):
        # Runs the code
        while not rospy.is_shutdown() and self.frontData == None and self.downData == None:
            rospy.logwarn("ERR: Missing data: /front/scan and /down/scan")
        while not rospy.is_shutdown() and self.frontData == None and self.downData != None:
            rospy.logwarn("ERR: Missing data: /front/scan (Front Lidar Not Connected)")
            self.convertToYDistAndZDistDown()
            self.getNumberOfObstaclesDown()
            self.sendMessagesDown()
            self.update_rate.sleep()
        while not rospy.is_shutdown() and self.downData == None and self.frontData != None:
            rospy.logwarn("ERR: Missing data: /down/scan (Down Lidar Not Connected)")
            self.convertToXDistAndYDistFront()
            self.getNumberOfObstaclesFront()
            self.sendMessagesFront()
            self.update_rate.sleep()
        while not rospy.is_shutdown():
            rospy.loginfo("Running")
            self.convertToXDistAndYDistFront()
            self.getNumberOfObstaclesFront()
            self.sendMessagesFront()
            self.convertToYDistAndZDistDown()
            self.getNumberOfObstaclesDown()
            self.sendMessagesDown()
            self.update_rate.sleep()

if __name__ == '__main__':
    obs = ObstacleDetection()
    obs.run()