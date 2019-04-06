#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msgs import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class GPSToOdom():
    def __init__(self):
        # Define variables
        self.gps_data = None
        self.g_frame_id = None
        self.odom_frame_id = None
        self.base_frame_id = None

        # Define ROS constructs
        rospy.init_node("gps_to_odom")
        self.gps_sub = rospy.Subscriber('/rtk_gps/fix', NavSatFix, self.gps_cb)
        self.odom_pub = rospy.Publisher('/gps/odom', Odometry, queue_size=10)
        self.update_rate = rospy.Rate(10)

    def gps_cb(self, msg):
        self.gps_data = msg

    def navsatfix_to_pose(self):
        pose_msg = PoseWithCovarianceStamped()

        pose_msg.header = self.gps_data.header
        pose_msg.header.frame_id = self.g_frame_id
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = z

          geometry_msgs::PoseWithCovarianceStampedPtr pose_msg(
      new geometry_msgs::PoseWithCovarianceStamped);


    def convert_to_odometry(self, pose_msg, twist_msg):
        odometry_msg = Odometry()

        odometry_msg.header.frame_id = self.odom_frame_id
        odometry_msg.child_frame_id = self.base_frame_id
        odometry_msg.pose = pose_msg
        odometry_msg.twist = twist_msg


    def run(self):
        # Takes no args, executes timed loop for node
        while not rospy.is_shutdown():
            if self.gps_data == None:
                rospy.loginfo('MSG: No GPS data published')
                self.update_rate.sleep()
                continue

            self.g_frame_id = get_param("~g_frame_id")
            self.odom_frame_id = get_param("~odom_frame_id")
            self.odom_frame_id = get_param("~base_frame_id")


            # linear = self.twist_data.linear.x
            # angular = self.twist_data.angular.z
            #
            # ack_msg = self.twist_to_ackermann(linear, angular)
            # self.ack_pub.publish(odom_msg)
            self.update_rate.sleep()

if __name__ == "__main__":
    rospy.set_param("~g_frame_id","world")
    rospy.set_param("~odom_frame_id","odom")
    rospy.set_param("~base_frame_id","base_link")

    gps_to_odom = GPSToOdom()
    gps_to_odom = run()






        #publish to /gps/odom

publisher = rospy.Publisher('/tractor_position', Pose, queue_size=10)
subscriber = rospy.Subscriber('/gps/rtkfix', Odometry, process_odom)
)
