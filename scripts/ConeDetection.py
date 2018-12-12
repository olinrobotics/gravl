#!/usr/bin/env python
######################################################################
 # KUBO Movement Behavior Node (Teensy 3.5)
 # @file ConeDetection.py
 # @author: Nathan Estill
 # @email: nathan.estill@students.olin.edu
 # @version: 2.0
 #
 # Sensing with the Lidar, detecting cones,
 # publishing their position
 ######################################################################
import rospy 
from math import *
import random
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan, PointCloud
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, Vector3, Point, Quaternion, Point32
from std_msgs.msg import ColorRGBA, Header
from ackermann_msgs.msg import AckermannDrive

class coneDetection():
    def __init__(self):
        rospy.init_node('ConeDetection', anonymous=True)
        self.frontlaserSub = rospy.Subscriber('/front/scan',LaserScan,self.frontLidarCB) # Subscribes to the front LIDAR, calls the callback function
        self.downToGround = 1.2 # distance to the ground
        self.threshold = 0.03 # distance that a point has to be close to the expected circle to be counted
        self.frontData = None # initializes the front laser data
        self.updateRate = rospy.Rate(5) # determines the speed
        self.angleOfIncline = 0.194724 # experimental angle that the front lidar is angled at
        self.radius = 0.25 # radius of the barrel, experimental but doesn't have to be too precise.
        self.visPubBarrels = rospy.Publisher('/BarrelCircles',Marker,queue_size=2)
        self.stalldaten = rospy.Publisher('/barrelCenters',PointCloud,queue_size=10)
        
    def frontLidarCB(self,data):
        self.frontData = data

    def convertDataTo3DPoints(self):
        self.frontPoints = []
        dataPoints = len(self.frontData.ranges)
        angleSweep = 190.0
        for i in range(len(self.frontData.ranges)): # Puts the tuple of data into x and y Distances
            angleRad = radians((i - dataPoints / 2) * (angleSweep / dataPoints))      
            xPoint = abs(cos(self.angleOfIncline) * cos(angleRad) * self.frontData.ranges[i]) # Computes the distance from the object
            yPoint = sin(angleRad) * self.frontData.ranges[i] # Computes the distance parallel to tractor
            zPoint = sin(self.angleOfIncline) * cos(angleRad) * self.frontData.ranges[i] # Computes the vertical distance to the ground
            self.frontPoints.append((xPoint,yPoint,zPoint))

    def determineNonGroundPoints(self):
        self.realPoints = []
        for i in self.frontPoints:
            if(i[2] < self.downToGround and i[2] > 0.1 and abs(i[1]) < 4.0 and i[0] < 5.0):
             # removes points that are farther than 5 meters from the front and 4 meters left or right from the center as well as points too close to the lidar
                self.realPoints.append(i)

    def pointsToCircle(self):
        centers = []
        self.points = []
        for i in range(800): # runs through 800 combinations, STILL NEEDS TO BE CALIBRATED, TOO HIGH NOW
            rand2Pts = random.sample(self.realPoints, 2) # selects 2 random points from the points in the range
            xCenter,yCenter = self.getCenterFrom2Points(rand2Pts) # gets a center of the circle from the 2 points
            if(xCenter != 0 or yCenter != 0):
                eligiblePoints = 0  
                for i in self.realPoints:
                    error = sqrt((i[0]-xCenter)**2 + (i[1]-yCenter)**2) - self.radius
                    if(-self.threshold < error and error < self.threshold):
                        eligiblePoints += 1
                if eligiblePoints > 20:
                    redundantCenter = False
                    for i in centers:
                        if((i[0]-xCenter)**2 + (i[1] - yCenter)**2 < 0.22**2):
                            redundantCenter = True
                    if(not redundantCenter):
                        centers.append((xCenter,yCenter,eligiblePoints))
                        p = Point32()
                        p.x = xCenter
                        p.y = yCenter
                        self.points.append(p)
        if(centers == []):
            return "Failure"
        else:

            return centers

    def getCenterRadiusFrom3Points(self,points):
        # Unused method to get the center and radius from 3 points
        x1 = points[0][0]
        y1 = points[0][1]
        x2 = points[1][0]
        y2 = points[1][1]
        x3 = points[2][0]
        y3 = points[2][1]
        xCenter = ((x1**2+y1**2)*(y2-y3) +(x2**2+y2**2)*(y3-y1) + (x3**2+y3**2)*(y1-y2)) / (2*(x1*(y2-y3)-y1*(x2-x3)+x2*y3-x3*y2))
        yCenter = ((x1**2+y1**2)*(x3-x2) +(x2**2+y2**2)*(x1-x3) + (x3**2+y3**2)*(x2-x1)) / (2*(x1*(y2-y3)-y1*(x2-x3)+x2*y3-x3*y2))
        radius = sqrt((xCenter-x1)**2+(yCenter-y1)**2)
        return (xCenter,yCenter,radius)

    def getCenterFrom2Points(self,points):
        x1 = points[0][0]
        y1 = points[0][1]
        x2 = points[1][0]
        y2 = points[1][1]
        r = self.radius
        q = sqrt((x2-x1)**2 + (y2-y1)**2)
        x3 = (x1+x2) / 2
        y3 = (y1+y2) / 2
        if(r**2 - (q/2)**2 < 0):
            return (0,0)
        xCenter1 = x3 + sqrt(r**2-(q/2)**2)*(y1-y2)/q
        yCenter1 = y3 + sqrt(r**2-(q/2)**2)*(x2-x1)/q 

        xCenter2 = x3 - sqrt(r**2-(q/2)**2)*(y1-y2)/q
        yCenter2 = y3 - sqrt(r**2-(q/2)**2)*(x2-x1)/q 
        
        if(xCenter1 > xCenter2):
            xCenter = xCenter1
            yCenter = yCenter1
        else:
            xCenter = xCenter2
            yCenter = yCenter2
        return (xCenter,yCenter)

    def display(self):
        barrelCircles = Marker() # marks the path that the tractor will take
        barrelCircles.header.frame_id = "laser"
        barrelCircles.header.stamp = rospy.Time.now()
        barrelCircles.scale = Vector3(0.02, 0.01, 0.01)
        barrelCircles.color = ColorRGBA(1,1,1,1) # white
        barrelCircles.type = 4 # makes the markers a line that is on every point, (line from 0-1, 1-2, 2-3, 3-4, 4-5, etc.
        for i in self.circles:
            xCenter = i[0]
            yCenter = i[1]
            x = 0.0 # initializes x, y, t
            y = 0.0
            t = 0.0
            r = self.radius
            while t < 2*pi: # stops at the sense range or if the angle gets above 90 degrees
                x = r * sin(t) + xCenter # parametric equation for a circle
                y = r * cos(t) + yCenter
                barrelCircles.points.append(Point(x,y,0))
                t += 0.01
        self.visPubBarrels.publish(barrelCircles)

    def publish(self):
        self.stalldatenTabelle = PointCloud()
        for i,p in enumerate(self.points):
            self.stalldatenTabelle.points.append(p)
        self.stalldaten.publish(self.stalldatenTabelle)

    def run(self):
        while not rospy.is_shutdown() and self.frontData == None:
            rospy.logwarn("ERR: Missing data: /front/scan")
        while not rospy.is_shutdown():
            self.convertDataTo3DPoints()
            self.determineNonGroundPoints()
            self.circles = self.pointsToCircle()
            self.publish()
            self.display()
            self.updateRate.sleep()


if __name__ == '__main__':
    cone = coneDetection()
    cone.run()