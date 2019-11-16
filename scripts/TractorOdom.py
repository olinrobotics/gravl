#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf

class TractorOdom:
    def __init__(self):
        rospy.init_node("tractor_odom")
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imuCB)
        self.gps_sub = rospy.Subscriber('/gps_odom', Odometry, self.gpsCB)
        self.odom_pub = rospy.Publisher('/tractor_odom', Odometry, queue_size=1)
        self.update_rate = rospy.Rate(10)

        self.odom_broadcaster = tf.TransformBroadcaster()

        self._imu_msg = Imu()
        self._gps_msg = Odometry()

    def imuCB(self, msg):
        self._imu_msg = msg

    def gpsCB(self, msg):
        self._gps_msg = msg

    def computeTractorOdom(self):
        msg = Odometry()
        msg.pose.pose.position = self._gps_msg.pose.pose.position
        msg.pose.pose.orientation = self._imu_msg.orientation
        self.odom_pub.publish(msg)

        # Broadcast message as tf & convert message units to meters
        self.odom_broadcaster.sendTransform((msg.pose.pose.position.x,
                                             msg.pose.pose.position.y,
                                             msg.pose.pose.position.z),
                                            (msg.pose.pose.orientation.w,
                                             msg.pose.pose.orientation.x,
                                             msg.pose.pose.orientation.y,
                                             msg.pose.pose.orientation.z),
                                            rospy.Time.now(),
                                            "/base_link",
                                            "/odom")

    def run(self):
        rospy.loginfo("Starting Tractor Odom")
        while not rospy.is_shutdown():
            self.computeTractorOdom()
            self.update_rate.sleep()

if __name__=="__main__":
    tOdom = TractorOdom()
    tOdom.run()
