#!/usr/bin/env python

# Import libraries
import rospy                                                                    # Basic ros functionality
import tf                                                                       # Ros transform frames
import numpy as np
from sensor_msgs.msg import PointCloud2                                         # Mapping data type
from sensor_msgs import point_cloud2                                            # Package for dealing with point clouds
from gravl.msg import Plane
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header, Float32MultiArray
debug = True                                                                    # Controls status printouts
visualize = True                                                                # Controls rviz display

class FeatureFinder():

    def init(self, debug, visualize):
        """ Sets up Feature Finder

            Args:
            self: reference to current object
            debug: boolean for debug printouts
            visualize: boolean for visualizing output

            Returns:
            none
            """

        # Status variables
        self.debug = debug
        self.visualize = visualize
        self.threshold = 0.08                                                   # Threshold above which lidar points are features, not road
        self.h_thresh = 1.2                                                     # Threshold above which lidar points are noise

        # Ros initialization
        rospy.init_node('feature_finder')                                       # Initialize node
        rospy.Subscriber("/laser_pointcloud", PointCloud2, self.pointcloud_cb)  # pointcloud2 subscriber
        self.feature_pub = rospy.Publisher("/pc_features", Float32MultiArray,
                                                            queue_size=10)      # feature publisher
        self.listener = tf.TransformListener()                                  # listen for laser - base_link transform
        if self.visualize:
            self.rviz_pub = rospy.Publisher('point_viz', PointStamped,
                                                            queue_size = 10)    # point-visualize publisher
            self.rviz_pub_pc = rospy.Publisher('pc_viz', PointCloud2,
                                                            queue_size = 10)    # pointcloud-visualize publisher

        # Attribute initialization
        self.transform = self.get_laser_transform()                             # tf from laser to base_link

    def show_rviz_marker(self, publisher, point):
        """ Shows given point in rviz
            Source: https://github.com/cse481sp17/cse481c/wiki/Lab-12:-Creating-Custom-Visualizations-in-RViz-using-Markers

            Args:
            self: reference to current object
            publisher: given publisher for marker objects to rviz
            point: point to represent

            Returns:
            none
            """

        print("MSG: Publishing point marker for plane")
        point_out = PointStamped(header=Header(frame_id="odom"),
                             point=Point(0.0,0.0,0.0))
        rospy.sleep(0.5)
        publisher.publish(point_out)
        return

    def get_laser_transform(self):

        """ Gets transform between laser and base_link

            Args:
            self: reference to current object

            Returns:
            tuple of translation and rotation
            """

        # Wait to get transform
        if self.debug: print("MSG: waiting for laser - base_link transform")
        self.listener.waitForTransform('laser', 'base_link', rospy.Time(0),
        rospy.Duration(10.0));
        if self.debug: print("MSG: transform received")

        # lookup transform
        try:
            (trans,rot) = self.listener.lookupTransform('laser', 'base_link',
            rospy.Time(0))
            return (trans,rot)

        except (tf.LookupException, tf.ConnectivityException,
        tf.ExtrapolationException):
            if self.debug: print("ERR: Lookup of laser - base_link transform failed")
            return

    def pointcloud_cb(self, data):
        """ Stores pointcloud data from subscriber in attribute

            Args:
            self: reference to current object
            data: pointcloud2 data from subscriber

            Returns:        print(self.transform)
            none
            """

        if self.debug: print("MSG: Received pointcloud")
        generator = point_cloud2.read_points(data)                              # Creates generator of pts from pointcloud
        self.analyze_scan(self.filter_lidar_scans(generator),
                        self.get_road_template(self.transform), self.threshold) # Analyzes cleaned lidar data in generator

        # Save data as array

    def analyze_scan(self, scan, road_template, thresh):
        """ Compares road scan to road template to find features

            Args:
            self: reference to current object
            scan: list of scan points
            road_template: plane representing goal road
            thresh: distance threshold for ground

            Returns:
            list of localized features
            """

        # Defines lists for road and feature points
        road_points = []
        feature_points = []
        for p in scan:
            if abs(p[2] - road_template.point[2]) < thresh:                    # Point is in road
                road_points.append(p[0:3])
            else:                                                               # Point is in feature
                feature_points.append(p[0:3])

        road_pc = point_cloud2.create_cloud_xyz32(header=Header(frame_id='odom'),
                                                        points=road_points)     # Creates pointcloud of points defining road plane
        #if self.visualize: self.rviz_pub_pc.publish(road_pc)                    # Visualize road pointcloud
        feature_plist = self.separate_features(feature_points)
        #feature_locs = self.locate_features(feature_plist)

    def get_road_template(self, transform):
        """Function to get desired road surface

            Args:
            self: reference to current object
            transform: transform from lidar to base_link

            Returns:
            Function/plane of desired road
            """

        point = (0,0,0) #transform[0]                                           # Position of plane w/ respect to lidar
        vector = [0,0,1]                                                        # Unit vector point up from ground
        plane = Plane(vector,point)                                             # Ros Plane defined with point and vector
        if self.visualize: self.show_rviz_marker(self.rviz_pub, plane.point)    # Visualizes road plane
        if self.debug: print("MSG: Road plane defined")
        return plane

    def analyze_pointcloud(self, pc, template):
        """ Analyzes pointcloud to find hills and holes

            Args:
            self: reference to current object
            pc: pointcloud2 object to analyze
            template: road template for comparison

            Returns:
            Array of arrays representing hills, Array of arrays representing holes
            """

    def separate_features(self, points):
        """ Separates point list of all feature points into individual features
            Uses RBNN Algorithm

            Args:
            self: reference to current object
            points: list of points in all features
            mas_feat: maximum number of features per scan

            Returns:
            List of lists representing individual features
            """
        radius = 0.3                                                           # Radius for Radially Bounded Nearest Neighbor
        features = []

        for p1 in points:                                                       # For all feature points
            has_feature_p1 = has_feature(features, p1)
            # if p1 is in feature:
                # next point
            # find nearest neighbors
            # for p2 in neighbor_pts:
                # if p1 and p2 are in features:
                    # if features are not the same:
                        # merge features
                # else if p2 has feature:
                    # p1 merges into p1 feature
                # else if p1 has feature:
                    # p2 merges into p1 feature
                # else if neither has feature:
                    # make new p1 feature
                    # put p2 in p1 feature
        # For all features
            # if less points than min for feature:
                # remove feature
        # return feature list



        if self.visualize:
            feature_pc = point_cloud2.create_cloud_xyz32(
                                header=Header(frame_id='odom'),points=p_in_rad)   # Creates pointcloud of points defining features
            self.rviz_pub_pc.publish(feature_pc)                                # Visualize feature pointcloud

    def has_feature(self, features, point):
        """ Checks for point in feature

            Args:
            self: reference to current object
            features: list of lists of points defining current features
            point: point to check for in features

            Returns:index of feature for point, else returns -1
            """
        index = -1
        for f in features:
            index += 1
            if point in f:
                return index

        return index

    def distance_3d(self, p1, p2):
        """ Determines euclidean distance between two points

            Args:
            self: reference to current object
            p1: 1st 3d point as tuple
            p2: 2nd 3d point as tuple

            Returns:distance as float
            """
        return np.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)

    def locate_feature(self, feature):
        """ Analyzes feature to determine spatial location

            Args:
            self: reference to current object
            feature: array of points representing object

            Returns:rotate a vector by a quaternion ros
            Array of info - [frontmost position, rearmost position, max/min height, hill/hole]
            """

    def filter_lidar_scans(self, generator):
        """ Removes lidar hood scan points from data

            Args:
            self: reference to current object
            generator: generator for points in lidar scan

            Returns:
            list of cleaned lidar points
            """

        return [p for p in generator if p[2] < self.h_thresh]


if __name__ == '__main__':
    ffinder = FeatureFinder()
    ffinder.init(debug, visualize)
    rospy.spin()
