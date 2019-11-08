#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry
from piksi_rtk_msgs.msg import BaselineNed

class PositionToOdom:
    def __init__(self):
        self.position_data = BaselineNed()

        rospy.init_node("position_to_odom")
        self.position_sub = rospy.Subscriber("/piksi/baseline_ned", BaselineNed,
                                             self.positionCB)
        self.odom_pub = rospy.Publisher("/gps_odom", Odometry, queue_size=1)
        self.update_rate = rospy.Rate(10)

        self.odom_broadcaster = tf.TransformBroadcaster()

    def positionCB(self, msg):
        self.position_data = msg

        self.odom_broadcaster.sendTransform((msg.n/1000., msg.e/1000., msg.d/1000.),
                  tf.transformations.quaternion_from_euler(0, 0, 0),
                  rospy.Time.now(),
                  "/base_link",
                  "/odom")

    def convertToOdometry(self):
        odom_msg = Odometry()
        odom_msg.header.frame_id = "/base_link"#TODO: Make this gps frame
        odom_msg.pose.pose.position.x = self.position_data.n/1000.
        odom_msg.pose.pose.position.y = self.position_data.e/1000.
        odom_msg.pose.pose.position.z = self.position_data.d/1000.
        return odom_msg

    def run(self):
        while not rospy.is_shutdown():
            self.odom_pub.publish(self.convertToOdometry())
            self.update_rate.sleep()

if __name__ == "__main__":
    p2o = PositionToOdom()
    p2o.run()
