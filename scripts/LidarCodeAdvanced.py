#!/usr/bin/env python3

"""Lidar safety behavior module.

Detects obstacles with lidar data using /scan topic, publishes True
to /estop topic if an obstacle is detected

@author: Nathan Estill
@email: nathan.estill@students.olin.edu
@version: 2.0
"""

import rospy
from math import *
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, Vector3, Point, Quaternion
from std_msgs.msg import ColorRGBA, Header
from ackermann_msgs.msg import AckermannDrive


class ObstacleDetection():
    """Obstacle detection behavior - uses front and downward lidar data."""

    def __init__(self):
        """Start up ObstacleDetection node, variables explained in wiki.

        https://github.com/olinrobotics/gravl/wiki/Vehicle-Safety
        """
        # State variables for obstacle detections
        self.hasSensedFront = False
        self.hasSensedDown = False

        # Attributes to store current data from subscribers
        self.frontData = None
        self.downData = None
        self.ackermannData = None

        # Configuration constants
        self.mode = rospy.get_param('~mode', "circle")
        self.senseRange = float(rospy.get_param('~senseRange', '5.0'))
        self.downToGround = 1.2  # Distance from down lidar to ground
        self.widthTractor = 1.25  # Kubo width (tire-tire)
        self.fbWheelDist = 1.524  # Kubo dist btw axles (front-back)
        self.angleOfIncline = 0.194724  # empirically Discovered, fix if needed
        self.threshold = 5  # How many points must be seen to trigger a stop?

        # Ros main setup
        rospy.init_node('ObstacleDetection', anonymous=True)
        rospy.Subscriber('/front/scan', LaserScan, self.frontLidarCB)
        rospy.Subscriber('/down/scan', LaserScan, self.downLidarCB)
        rospy.Subscriber('/cmd_vel', AckermannDrive, self.ackermannCB)

        self.update_rate = rospy.Rate(5)  # Behavior loop rate is 5 Hz
        self.pubEstop = rospy.Publisher('/softestop', Bool,  queue_size=10)

        # Ros visualization setup
        self.vis_pubFront = rospy.Publisher(
            '/LidarFront_pt', Marker, queue_size=2)
        self.vis_pubDown = rospy.Publisher(
            '/LidarLineDown_pt', Marker, queue_size=2)
        self.vis_pubLines = rospy.Publisher(
            '/TractorLine_pt', Marker, queue_size=2)

    def frontLidarCB(self, data):
        """Store data from front-facing lidar."""
        self.frontData = data

    def downLidarCB(self, data):
        """Store data from downward-facing lidar."""
        self.downData = data

    def ackermannCB(self, data):
        """Store data on current wheel angle & velocity."""
        self.ackermannData = data

    def convertToXDistAndYDistFront(self):
        """Convert raw front scan data to xyz dist of each pt from Kubo."""
        # Create arrays for storage
        self.totalDistFront = []
        self.xDistFront = []
        self.yDistFront = []
        self.zDistFront = []
        dataPoints = len(self.frontData.ranges)
        angleSweep = 190.0

        # Transform each lidar point
        for i in range(len(self.frontData.ranges)):
            self.totalDistFront.append(self.frontData.ranges[i])
            angleRad = radians(
                (i - dataPoints / 2) * (angleSweep / dataPoints))
            xDist = abs(
                cos(self.angleOfIncline) * cos(angleRad)
                * self.totalDistFront[i])
            yDist = sin(angleRad) * self.totalDistFront[i]
            zDist = sin(self.angleOfIncline) * cos(angleRad)\
                * self.totalDistFront[i]

            self.xDistFront.append(xDist)
            self.yDistFront.append(yDist)
            self.zDistFront.append(zDist)

    def getNumberOfObstaclesFront(self, visualize):
        """Calculate number of points that pose a threat to the tractor."""
        # If wheel data not found, assume steering straight
        wheelAngle = 0 if self.ackermannData is None\
            else self.ackermannData.steering_angle

        self.obstaclePointsFront = 0  # Number of points that aren't ground
        self.triggerPointsFront = 0  # Number of points breaking threshold

        # If checking straight in front of tractor
        if self.mode == "line" or wheelAngle == 0:
            for i in range(len(self.totalDistFront)):  # Sweep point distances

                # Increment obstacle counter if point isn't on ground
                if (sqrt(self.xDistFront[i]**2 + self.yDistFront[i]**2)
                        < self.senseRange):
                    self.obstaclePointsFront += 1

                    # Increment trigger counter if point will hit tractor
                    if(sin(wheelAngle) * self.xDistFront[i]
                            - self.widthTractor / 2.0
                            < self.yDistFront[i]
                            < sin(wheelAngle) * self.xDistFront[i]
                            + self.widthTractor / 2.0):
                        self.triggerPointsFront += 1

        # If checking along tractor trajectory
        elif self.mode == "circle":
            r = self.fbWheelDist / sin(wheelAngle)  # Path radius a/sin(theta)

            # Set inner and outer radii based on sign of steer angle
            # Note: radius slightly different from above
            if(wheelAngle > 0):
                rInner = r - self.widthTractor / 2
                rOuter = r + self.widthTractor / 2
            elif(wheelAngle < 0):
                rInner = r + self.widthTractor / 2
                rOuter = r - self.widthTractor / 2

            for i in range(len(self.totalDistFront)):  # Sweep point distances

                # Increment obstacle counter if point isn't on ground
                if (sqrt(self.xDistFront[i]**2 + self.yDistFront[i]**2)
                        < self.senseRange):
                    self.obstaclePointsFront += 1

                    # Increment trigger counter if point is in path
                    if (rInner**2
                            < self.xDistFront[i]**2 + (self.yDistFront[i]+r)**2
                            < rOuter**2):
                        self.triggerPointsFront += 1

        if visualize:
            lidarPoints = Marker()  # Shows lidar pts
            # TODO verify that this is the correct frame
            lidarPoints.header.frame_id = "laser"
            lidarPoints.header.stamp = rospy.Time.now()
            lidarPoints.type = 8  # Sets type to point list
            lidarPoints.scale = Vector3(0.02, 0.02, 0.02)  # Approximately 2cm
            lidarPoints.color = ColorRGBA(0, 0, 1, 1)  # Blue

            # Add points to lidarPoints and publish
            for i in range(len(self.totalDistFront)):
                lidarPoints.points.append(Point(
                    self.xDistFront[i], self.yDistFront[i],
                    -self.zDistFront[i]))
            self.vis_pubFront.publish(lidarPoints)

    def sendMessagesFront(self):
        """Estop tractor if trigger point count passes threshold."""
        # Stop tractor if threshold crossed (low -> high)
        if (self.triggerPointsFront > self.threshold
                and not self.hasSensedFront):
            self.pubEstop.publish(True)
            self.hasSensedFront = True

        # Stop tractor if threshold crossed (high -> low)
        if (self.triggerPointsFront <= self.threshold
                and self.hasSensedFront):
            self.pubEstop.publish(False)
            self.hasSensedFront = False

    def convertToYDistAndZDistDown(self):
        """Convert raw downward scan data to yz dist of each pt from Kubo."""
        # Create arrays for storage
        self.totalDistDown = []
        self.xDistDown = []
        self.yDistDown = []
        self.zDistDown = []
        dataPoints = len(self.downData.ranges)
        angleSweep = 270.0

        # Transform each lidar point
        for i in range(len(self.downData.ranges)):
            self.totalDistDown.append(self.downData.ranges[i])
            angleRad = radians(
                (i - dataPoints / 2) * (angleSweep / dataPoints))
            zDist = cos(angleRad) * self.totalDistDown[i]
            yDist = sin(angleRad) * self.totalDistDown[i]

            self.zDistDown.append(zDist)
            self.yDistDown.append(yDist)

    def getNumberOfObstaclesDown(self, visualize):
        """Calculate number of points that pose a threat to the tractor."""
        # Mode doesn't matter bc downward lidar is so close to tires

        self.obstaclePointsDown = 0  # Number of points that aren't ground
        self.triggerPointsDown = 0  # Number of points breaking threshold

        for i in range(len(self.totalDistDown)):  # Sweep point distances

            # Increment obstacle counter if point isn't on ground
            if (0.1 < self.zDistDown[i] < self.downToGround):
                self.obstaclePointsDown += 1

                # Increment trigger counter if point will hit tractor
                if (abs(self.yDistDown[i]) < (self.widthTractor / 2.0)):
                    self.triggerPointsDown += 1

        if visualize:
            lidarPoints = Marker()  # Shows lidar pts
            # TODO verify frame is correct
            lidarPoints.header.frame_id = "laser"
            lidarPoints.header.stamp = rospy.Time.now()
            lidarPoints.type = 8  # Sets type to point list
            lidarPoints.scale = Vector3(0.02, 0.02, 0.02)  # Approximately 2cm
            lidarPoints.color = ColorRGBA(0, 0, 1, 1)  # Blue

            # Add points to lidarPoints and publish
            for i in range(len(self.totalDistDown)):
                lidarPoints.points.append(Point(
                    0, self.yDistDown[i], -self.zDistDown[i]))
            self.vis_pubDown.publish(lidarPoints)

    def sendMessagesDown(self):
        """Estop tractor if trigger point count passes threshold."""
        # Stop tractor if threshold crossed (low -> high)
        if (self.triggerPointsDown > self.threshold
                and not self.hasSensedDown):
            self.pubEstop.publish(True)
            self.hasSensedDown = True

        # Start tractor if threshold crossed (high -> low)
        if (self.triggerPointsDown <= self.threshold
                and self.hasSensedDown):
            self.pubEstop.publish(False)
            self.hasSensedDown = False

    def publishTractorLines(self):
        """Visualize path of tractor tires based on mode."""
        tractorLines = Marker()  # Marks planned tractor path
        tractorLines.header.frame_id = "laser"
        tractorLines.header.stamp = rospy.Time.now()
        tractorLines.scale = Vector3(0.02,  0.01,  0.01)
        tractorLines.color = ColorRGBA(1, 1, 1, 1)  # White

        # Visualization for line mode
        if self.mode == "line":
            # Marker type connects every two points (line from 0-1, 2-3, 4-5)
            tractorLines.type = 5

            # If wheel data not found, assume steering straight
            wheelAngle = 0 if self.ackermannData is None\
                else self.ackermannData.steering_angle

            # Line for right wheel
            tractorLines.points.append(Point(0, self.widthTractor / 2, 0))
            tractorLines.points.append(Point(
                self.senseRange * cos(wheelAngle),
                self.widthTractor / 2 + sin(wheelAngle) * self.senseRange, 0))

            # Line for left wheel
            tractorLines.points.append(Point(0, -self.widthTractor / 2, 0))
            tractorLines.points.append(Point(
                self.senseRange * cos(wheelAngle),
                -self.widthTractor / 2 + sin(wheelAngle) * self.senseRange, 0))

            self.vis_pubLines.publish(tractorLines)

        # Visualization for arc mode
        elif self.mode == "circle":

            # If wheel data not found or steering straight, make straight lines
            if (self.ackermannData is None
                    or self.ackermannData.steering_angle == 0.0):

                # Marker type connects every other pair of points (0-1, 2-3)
                tractorLines.type = 5
                tractorLines.points.append(Point(
                    100, self.widthTractor / 2, 0))
                tractorLines.points.append(Point(
                    -100, self.widthTractor / 2, 0))
                tractorLines.points.append(Point(
                    -100, -self.widthTractor / 2, 0))
                tractorLines.points.append(Point(
                    100, -self.widthTractor / 2, 0))

            # Generate arced lines
            else:
                wheelAngle = self.ackermannData.steering_angle
                tractorLines.type = 4  # Marker type connects every pair of pts
                x, y, t = 0.0, 0.0, 0.0
                r = self.fbWheelDist / sin(wheelAngle)

                # Generate points until sense range or angle > 90 degrees
                while x**2 + y**2 < self.senseRange**2 and t < pi / 2:
                    rOuter = self.fbWheelDist / sin(wheelAngle)\
                        + self.widthTractor / 2
                    x = abs(rOuter * sin(t))  # Parametric equation for circle
                    y = rOuter * cos(t) - r
                    if x**2 + y**2 < self.senseRange**2:
                        tractorLines.points.append(Point(x, y, 0))
                    t += 0.01

                while t >= -0.01:
                    rInner = self.fbWheelDist / sin(wheelAngle)\
                        - self.widthTractor / 2
                    x = abs(rInner * sin(t))  # Parametric equation for circle
                    y = rInner * cos(t) - r
                    if(x**2 + y**2 < self.senseRange**2):
                        tractorLines.points.append(Point(x, y, 0))
                    t -= 0.01

            self.vis_pubLines.publish(tractorLines)

    def run(self, visualize=False):
        """Run main behavior loop, dependent on available data."""
        # If missing data from both Lidar
        while (not rospy.is_shutdown()
                and self.frontData is None
                and self.downData is None):
            rospy.logwarn_throttle(
                5, "Missing data: /front/scan and /down/scan")

        # If missing data from front Lidar only
        while (not rospy.is_shutdown()
                and self.frontData is None
                and self.downData is not None):
            rospy.logwarn_throttle(5, "Missing data: /front/scan")

            self.convertToYDistAndZDistDown()
            self.getNumberOfObstaclesDown(visualize)
            self.sendMessagesDown()
            if visualize:
                self.publishTractorLines()
            self.update_rate.sleep()

        # If missing data from down Lidar only
        while (not rospy.is_shutdown()
                and self.downData is None
                and self.frontData is not None):
            rospy.logwarn_throttle(5, "Missing data: /down/scan")
            self.convertToXDistAndYDistFront()
            self.getNumberOfObstaclesFront(visualize)
            self.sendMessagesFront()
            if visualize:
                self.publishTractorLines()
            self.update_rate.sleep()
        rospy.loginfo("Running")

        # If all data is available
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
