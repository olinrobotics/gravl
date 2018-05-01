#!/usr/bin/env python

# Import libraries
import rospy                                                                    # Basic ros functionality
import tf                                                                       # Ros transform frames
from sensor_msgs.msg import PointCloud2                                         # Mapping data type
from std_msgs.msg import Float32MultiArray                                      # Feature data type
debug = True                                                                    # Controls status printouts
visualize = True                                                                # Controls rviz display

class FeatureFinder():

    def init(self, debug):
        """ Sets up Feature Finder

            Args:
            self: reference to current object

            Returns:
            none
            """

        # Ros initialization
        rospy.init_node('feature_finder')                                       # Initialize node
        rospy.Subscriber("/laser_pointcloud", PointCloud2, self.pointcloud_cb)  # pointcloud2 subscriber
        rospy.Publisher("/pc_features", Float32MultiArray, queue_size=10)       # feature publisher
        self.listener = tf.TransformListener()                                  # listen for laser - base_link transform

        # Attribute initialization
        self.debug = debug
        self.visualize = visualize
        if self.visualize: self.transform = self.get_laser_transform()

    def get_laser_transform(self):
        """ Gets transform between laser and base_link

            Args:
            self: reference to current object

            Returns:
            none
            """

        # Wait to get transform
        if self.debug: print("MSG: waiting for laser - base_link transform")
        self.listener.waitForTransform('laser', 'base_link', rospy.Time(0),
        rospy.Duration(10.0));
        if self.debug: print("MSG: transform received")

        # lookup transform
        try:
            (trans,rot) = self.listener.lookupTransform('imu', 'base_link',
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

            Returns:
            none
            """



        if self.debug:
            print("MSG: Received pointcloud")
            print (data.height)
            print (data.width)
        # Save data as array

    def analyze_road(self):
        """ Analyzes road to get positions of features

            Args:
            self: reference to current object

            Returns:
            list of localized features
            """

    def get_road_template(self):
        """Function to get desired road surface

            Args:
            self: reference to current object

            Returns:
            Function/plane of desired road
            """

    def analyze_pointcloud(self, pc, template):
        """ Analyzes pointcloud to find hills and holes

            Args:
            self: reference to current object
            pc: pointcloud2 object to analyze
            template: road template for comparison

            Returns:
            Array of arrays representing hills, Array of arrays representing holes
            """

    def locate_feature(self, feature):
        """ Analyzes feature to determine spatial location

            Args:
            self: reference to current object
            feature: array of points representing object

            Returns:
            Array of info - [frontmost position, rearmost position, max/min height, hill/hole]
            """

if __name__ == '__main__':
    ffinder = FeatureFinder()
    ffinder.init(debug)
    rospy.spin()
