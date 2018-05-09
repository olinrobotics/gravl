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
        self.thresh_road = 0.08                                                   # Threshold above which lidar points are features, not road
        self.thresh_h = 1.2                                                     # Threshold above which lidar points are noise
        self.thresh_min_feat = 10                                               # Threshold below which features are too small
        self.thresh_rad = 0.3                                                   # Threshold above which point is in separate feature

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
                        self.get_road_template(self.transform), self.thresh_road) # Analyzes cleaned lidar data in generator

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
        feature_plist = self.separate_features(feature_points, self.thresh_min_feat, self.thresh_rad)
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

    def separate_features(self, points, min_feat, radius):
        """ Separates point list of all feature points into individual features
            Uses RBNN Algorithm

            Args:
            self: reference to current object
            points: list of points in all features
            max_feat: maximum number of features per scan
            radius: radius for radially bounded nearest neighbor

            Returns:
            List of lists representing individual features
            """
        features = []

        for p1 in points:                                                       # For all feature points
            has_feature_p1 = self.has_feature(features, p1)

            # find nearest neighbors
            p1_neighbors = [p for p in points if self.distance_3d(p1,p) < self.thresh_h]

            for p2 in p1_neighbors:                                             # For all neighbor points
                has_feature_p2 = self.has_feature(features, p2)
                if has_feature_p1 > -1 and has_feature_p2 > -1:                 # If both points are in features

                    if has_feature_p1 != has_feature_p2:                        # If features are not the same

                        # Merge features
                        features[has_feature_p1] = features[has_feature_p1] + features[has_feature_p2]
                        del features[has_feature_p2]

                elif has_feature_p1 == -1 and has_feature_p2 > -1:              # If only p2 has feature
                    features[has_feature_p2].append(p1)                         # Add p1 to p2 feature

                elif has_feature_p1 > -1 and has_feature_p2 == -1:              # If only p1 has feature
                    features[has_feature_p1].append(p2)                         # Add p2 to p1 feature

                elif has_feature_p1 == has_feature_p2 == -1:                    # If neither point has feature
                    features.append([p1,p2])                                    # Make new feature with p1 & p2

        print(len(features))
        for feat in features:
            if len(feat) < min_feat:                                            # if feature smaller than threshold
                del features[feat]                                              # remove feature
                if self.debug: print('MSG: Deleted too small feature')

            if self.visualize:
                feature_pc = point_cloud2.create_cloud_xyz32(
                                header=Header(frame_id='odom'),points=feat)     # Creates pointcloud of points defining features
                self.rviz_pub_pc.publish(feature_pc)                            # Visualize feature pointcloud

        return features





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

        return [p for p in generator if p[2] < self.thresh_h]


if __name__ == '__main__':
    ffinder = FeatureFinder()
    ffinder.init(debug, visualize)
    rospy.spin()
