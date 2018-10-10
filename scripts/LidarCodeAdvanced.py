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
from math import *
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, Vector3, Point, Quaternion
from std_msgs.msg import ColorRGBA, Header
from ackermann_msgs.msg import AckermannDrive


class ObstacleDetection():

    def __init__(self):
        # Variables explained in Github Wiki: https://github.com/olinrobotics/gravl/wiki/Vehicle-Safety
        rospy.init_node('ObstacleDetection', anonymous=True)
        self.hasSensedFront = False # If there is an obstacle sensed
        self.hasSensedDown = False
        self.pubEstop = rospy.Publisher('/softestop', Bool,  queue_size=10) # Publisher that publishes to the estop
        self.frontlaserSub = rospy.Subscriber('/front/scan',LaserScan,self.frontLidarCB) # Subscribes to the front LIDAR, calls the callback function
        self.downLaserSub = rospy.Subscriber('/down/scan',LaserScan,self.downLidarCB) # Subscribes to the down Lidar, calls the cb function
        self.ackermannSub = rospy.Subscriber('/cmd_vel',AckermannDrive,self.ackermannCB)
        self.frontData = None   
        self.downData = None
        self.ackermannData = None
        self.DownToGround = 1.2 # Distance from down lidar to the ground
        self.senseRange = float(rospy.get_param('~senseRange','5.0'))
        self.mode = rospy.get_param('~mode',"circle")
        self.widthTractor = 1.25 # Horizontal length of the tractor
        self.fbWheelDist = 1.524 # Measures the distance from the front wheels to the back wheels
        self.threshold = 5 # How many points must be seen to trigger a stop?
        self.update_rate = rospy.Rate(5)
        self.vis_pubFront = rospy.Publisher('/LidarFront_pt', Marker, queue_size=2)
        self.vis_pubDown = rospy.Publisher('/LidarLineDown_pt', Marker, queue_size=2)
        self.vis_pubLines = rospy.Publisher('/TractorLine_pt',Marker,queue_size=2)


    def frontLidarCB(self,data):
        # Collects data from the Front Hokuyo Lidar and stores it
        self.frontData = data

    def downLidarCB(self,data):
        # Collects data from the Down Hokuyo Lidar and stores it
        self.downData = data

    def ackermannCB(self,data):
        self.ackermannData = data

    def convertToXDistAndYDistFront(self):
        self.totalDistFront = [] # Initializing arrays
        self.xDistFront = [] # Distance from front of tractor
        self.yDistFront = [] # Distance from center of tractor
        self.zDistFront = [] # Distance from the tractor to the point, (usually the ground)
        dataPoints = len(self.frontData.ranges)
        angleSweep = 190.0
        angleOfIncline = 0.194724 # ?Empirically Discovered. Fix if necessary
        for i in range(len(self.frontData.ranges)): # Puts the tuple of data into x and y Distances
            self.totalDistFront.append(self.frontData.ranges[i])
            angleRad = radians((i - dataPoints / 2) * (angleSweep / dataPoints))      
            self.xDistFront.append(abs(cos(angleOfIncline) * cos(angleRad) * self.totalDistFront[i])) # Computes the distance from the object
            self.yDistFront.append(sin(angleRad) * self.totalDistFront[i]) # Computes the distance parallel to tractor
            self.zDistFront.append(sin(angleOfIncline) * cos(angleRad) * self.totalDistFront[i])
            #rospy.loginfo(str(angleRad) + ": " + str(self.zDistFront[i]))

    def getNumberOfObstaclesFront(self,visualize):
        # Calculates the number of points that pose a threat to the tractor
        if self.ackermannData == None:
            wheelAngle = 0
        else:
            wheelAngle = self.ackermannData.steering_angle
        self.obstaclePointsFront = 0 # Counts how many points are not the ground
        self.triggerPointsFront = 0 # Counts number of points breaking threshold  
        if self.mode == "line" or wheelAngle == 0:       
            for i in range(len(self.totalDistFront)): # Sweep through the distances
                if(sqrt(self.xDistFront[i]**2 + self.yDistFront[i]**2) < self.senseRange): # Is there an object that is not the ground?
                    self.obstaclePointsFront += 1 # Add a point into the number of obstacle points
                    if((sin(wheelAngle) * self.xDistFront[i] - self.widthTractor / 2.0) < self.yDistFront[i] < sin(wheelAngle) * self.xDistFront[i] + self.widthTractor / 2.0): #Will the obstacle hit the tractor?
                        self.triggerPointsFront += 1 # Add a point the the number of triggers
        elif self.mode == "circle":
            r = self.fbWheelDist / sin(wheelAngle)
            if(wheelAngle > 0):
                rInner = self.fbWheelDist / sin(wheelAngle) - self.widthTractor / 2
                rOuter = self.fbWheelDist / sin(wheelAngle) + self.widthTractor / 2
            elif(wheelAngle < 0):
                rInner = self.fbWheelDist / sin(wheelAngle) + self.widthTractor / 2
                rOuter = self.fbWheelDist / sin(wheelAngle) - self.widthTractor / 2
            for i in range(len(self.totalDistFront)):
                if sqrt(self.xDistFront[i]**2 + self.yDistFront[i]**2 < self.senseRange):
                    self.obstaclePointsFront += 1
                    if (rInner**2 < self.xDistFront[i]**2 + (self.yDistFront[i]-r)**2 < rOuter**2):
                        self.triggerPointsFront += 1
        if visualize:
            lidarPoints = Marker() # Marker to visualize used lidar pts
            lidarPoints.header.frame_id = "laser" # Publishes it to the laser link, idk if it should be changed
            lidarPoints.header.stamp = rospy.Time.now()
            lidarPoints.type = 8 # makes it a list of points
            lidarPoints.scale = Vector3(0.02,0.02,0.02) # scale is about 2 cm
            lidarPoints.color = ColorRGBA(0,0,1,1) # Color is blue
            for i in range(len(self.totalDistFront)):
                lidarPoints.points.append(Point(self.xDistFront[i],self.yDistFront[i],-self.zDistFront[i]))
            self.vis_pubFront.publish(lidarPoints)

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
            angleRad = radians((i - dataPoints / 2) * (angleSweep / dataPoints))
            self.zDistDown.append(cos(angleRad) * self.totalDistDown[i]) # Computes the distance from the object
            self.yDistDown.append(sin(angleRad) * self.totalDistDown[i]) # Computes the distance parallel to tractor

    def getNumberOfObstaclesDown(self,visualize):
        # Calculates the number of points that pose a threat to the tractor
        self.obstaclePointsDown = 0 # Counts how many points are not the ground
        self.triggerPointsDown = 0 # Counts number of points breaking threshold
        for i in range(len(self.totalDistDown)): # Sweep through the distances
            if(self.zDistDown[i] < self.DownToGround and self.zDistDown[i] > 0.1): # Is there an object that is not the ground?
                self.obstaclePointsDown += 1 # Add a point into the number of obstacle points
                if(abs(self.yDistDown[i]) < (self.widthTractor / 2.0)): #Will the obstacle hit the tractor?
                    self.triggerPointsDown += 1 # Add a point the the number of triggers
        if visualize:
            lidarPoints = Marker() # Marker to visualize used lidar pts
            lidarPoints.header.frame_id = "laser" # Publishes it to the laser link, idk if it should be changed
            lidarPoints.header.stamp = rospy.Time.now()
            lidarPoints.type = 8 # makes it a list of points
            lidarPoints.scale = Vector3(0.02,0.02,0.02) # scale is about 2 cm
            lidarPoints.color = ColorRGBA(0,0,1,1) # Color is Boolue
            for i in range(len(self.totalDistDown)):
                lidarPoints.points.append(Point(0,self.yDistDown[i],-self.zDistDown[i]))
            self.vis_pubDown.publish(lidarPoints)
            
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

    def publishTractorLines(self):
        if self.mode == "line":
            tractorLines = Marker()
            tractorLines.header.frame_id = "laser"
            tractorLines.header.stamp = rospy.Time.now()
            tractorLines.type = 5
            tractorLines.scale = Vector3(0.02, 0.01, 0.01)
            tractorLines.color = ColorRGBA(1,1,1,1)
            if self.ackermannData == None:
                tractorLines.points.append(Point(100,self.widthTractor / 2,0))
                tractorLines.points.append(Point(-100,self.widthTractor / 2,0))
                tractorLines.points.append(Point(-100,-self.widthTractor / 2,0))
                tractorLines.points.append(Point(100,-self.widthTractor / 2,0))
            else:
                wheelAngle = self.ackermannData.steering_angle
                tractorLines.points.append(Point(self.senseRange * cos(wheelAngle),self.widthTractor / 2 + sin(wheelAngle) * self.senseRange,0))
                tractorLines.points.append(Point(0,self.widthTractor / 2,0))
                tractorLines.points.append(Point(0,-self.widthTractor / 2,0))
                tractorLines.points.append(Point(self.senseRange * cos(wheelAngle),-self.widthTractor / 2 + sin(wheelAngle) * self.senseRange,0))
            self.vis_pubLines.publish(tractorLines)
        elif self.mode == "circle":
            tractorLines = Marker()
            tractorLines.header.frame_id = "laser"
            tractorLines.header.stamp = rospy.Time.now()
            tractorLines.scale = Vector3(0.02, 0.01, 0.01)
            tractorLines.color = ColorRGBA(1,1,1,1)
            if self.ackermannData == None or self.ackermannData.steering_angle == 0.0:
                tractorLines.type = 5
                tractorLines.points.append(Point(100,self.widthTractor / 2,0))
                tractorLines.points.append(Point(-100,self.widthTractor / 2,0))
                tractorLines.points.append(Point(-100,-self.widthTractor / 2,0))
                tractorLines.points.append(Point(100,-self.widthTractor / 2,0))
            else:
                wheelAngle = self.ackermannData.steering_angle
                print(wheelAngle)
                tractorLines.type = 4       
                x = 0
                y = 0
                t = 0
                r = self.fbWheelDist / sin(wheelAngle)
                while x**2 + y**2 < self.senseRange**2 and t < pi / 2:
                    rOuter = self.fbWheelDist / sin(wheelAngle) + self.widthTractor / 2
                    x = abs(rOuter * sin(t))
                    y = rOuter * cos(t) - r
                    if x**2 + y**2 < self.senseRange **2:
                        tractorLines.points.append(Point(x,y,0))
                    t += 0.01
                while t >= -0.01:
                    rInner = self.fbWheelDist / sin(wheelAngle) - self.widthTractor / 2
                    x = abs(rInner * sin(t))
                    y = rInner * cos(t) - r
                    if(x**2 + y**2 < self.senseRange**2):
                        tractorLines.points.append(Point(x,y,0))
                    t -= 0.01
            self.vis_pubLines.publish(tractorLines)

    def run(self,visualize = False):
        # Runs the code
        while not rospy.is_shutdown() and self.frontData == None and self.downData == None:
            rospy.logwarn("ERR: Missing data: /front/scan and /down/scan")
        while not rospy.is_shutdown() and self.frontData == None and self.downData != None:
            rospy.logwarn("ERR: Missing data: /front/scan (Front Lidar Not Connected)")
            self.convertToYDistAndZDistDown()
            self.getNumberOfObstaclesDown(visualize)
            self.sendMessagesDown()
            if visualize:
                self.publishTractorLines()
            self.update_rate.sleep()
        while not rospy.is_shutdown() and self.downData == None and self.frontData != None:
            rospy.logwarn("ERR: Missing data: /down/scan (Down Lidar Not Connected)")
            self.convertToXDistAndYDistFront()
            self.getNumberOfObstaclesFront(visualize)
            self.sendMessagesFront()
            if visualize:
                self.publishTractorLines()
            self.update_rate.sleep()
        rospy.loginfo("Running")
        while not rospy.is_shutdown():
            self.convertToXDistAndYDistFront()
            self.getNumberOfObstaclesFront(visualize)
            self.sendMessagesFront()
            self.convertToYDistAndZDistDown()
            self.getNumberOfObstaclesDown(visualize)
            self.sendMessagesDown()
            if visualize:
                self.publishTractorLines()
            self.update_rate.sleep()

if __name__ == '__main__':
    obs = ObstacleDetection()
    obs.run(True)