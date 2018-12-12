#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist, Vector3, Point, Quaternion, Point32, PoseStamped, Pose
from std_msgs.msg import ColorRGBA, Header

class Gradient():
    def __init__(self):
        rospy.init_node('Gradient', anonymous=True)
        self.barrelSub = rospy.Subscriber('/barrelCenters',PointCloud,self.barrelCB)
        self.goalSub = rospy.Subscriber('/move_base_simple/goal',PoseStamped,self.goalPointCB)
        self.barrelPoints = np.array([[10,10]])
        self.vectorVisPub = rospy.Publisher('/vector_vis',Marker,queue_size=2)
        self.goalPoint = None

    def barrelCB(self,data):
        self.barrelPoints = []
        for i in data.points:
            self.barrelPoints.append([i.x,i.y])
        self.barrelPoints = np.array(self.barrelPoints)

    def goalPointCB(self,data):
        self.goalPoint = np.array([data.pose.position.x, data.pose.position.y])

    def potential(self):
        '''
        Returns gradient of potential field at origin.
        @param self.barrelPoints a nx2 numpy array with points as rows.
        @param self.goalPoint a single dimensional numpy array with the self.goalPoint x, y
        @return 1D numpy array with the gradient at the origin
        '''

        dx = 0.5
        samples = 3
        center = samples // 2
        lattice = (np.arange(samples) - center) * dx
        mesh = np.array(np.meshgrid(lattice, lattice))

        repelSfc = np.log(np.linalg.norm(np.repeat(
            mesh[np.newaxis], self.barrelPoints.shape[0], 0).transpose(3, 2, 0, 1) - self.barrelPoints, axis=-1)).sum(2)
        attrSfc = -10*np.log(np.linalg.norm(self.goalPoint - mesh.T, axis=2))

        gradients = np.array(np.gradient(repelSfc + attrSfc, dx))

        return gradients[:, center, center]

    def display(self,vector):
        vectorMarker = Marker() # marks the path that the tractor will take
        vectorMarker.header.frame_id = "laser"
        vectorMarker.header.stamp = rospy.Time.now()
        vectorMarker.scale = Vector3(0.02,0.10,0.2)
        vectorMarker.points.append(Point(0,0,0))
        vectorMarker.points.append(Point(vector[0],vector[1],0))
        vectorMarker.color = ColorRGBA(0,1,1,1) # white
        vectorMarker.type = 0 # makes the markers a line that is on every point, (line from 0-1, 1-2, 2-3, 3-4, 4-5, etc.
        self.vectorVisPub.publish(vectorMarker)

    def run(self):
        while not rospy.is_shutdown():
            if(self.goalPoint == None):
                vector = np.array([0,0])
            else:
                vector = self.potential()
                rospy.loginfo(vector)
                self.display(vector)

if __name__ == '__main__':
    grad = Gradient()
    grad.run()