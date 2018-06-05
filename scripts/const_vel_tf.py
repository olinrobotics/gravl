#!/usr/bin/env python
import roslib
import rospy
import tf#!/usr/bin/env python
import roslib
import rospy
import tf
from sensor_msgs.msg import Imu             # Message for Imu data
from geometry_msgs.msg import Quaternion    # Message for orientation data
from geometry_msgs.msg import PoseStamped   # Message for position data
from tf.transformations import quaternion_from_euler

class Tfbroadcaster():

    def init(self):

        # ROS initialization
        rospy.init_node('const_vel_tf')                  # Initialized tf node
        self.broadcaster = tf.TransformBroadcaster()            # broadcaster for baselink - odom transform

        # Initialize state variables
        self.pos = [0.0,0.0,0.0]
        self.vel = [0.6,0.0,0.0]                    # Speed for tractor at low gear, low RPM, full pedal
        self.orientation = quaternion_from_euler(0.0,0.0,0.0)

        self.update_rate = 10.0                     # Hz
        self.rate = rospy.Rate(self.update_rate)    # tf publish rate


    # position update function, updates position based on velocity and rate at which it is called
    def update_pos(self,pos, vel,rate):
        pos[0] += vel[0]/rate
        pos[1] += vel[1]/rate
        pos[2] += vel[2]/rate
        return pos

    def main(self):

        while not rospy.is_shutdown():

            # lookup specific transform

            # Sent transform
            pose = PoseStamped()
            pose.pose.position.x = self.pos[0]
            pose.pose.position.y = self.pos[1]
            pose.pose.position.z = self.pos[2]
            pose.header.frame_id = 'base_link'
            self.broadcaster.sendTransform((self.pos[0], self.pos[1], self.pos[2]),
                             self.orientation,
                             rospy.Time.now(),
                             'base_link',
                             'odom')
            self.pos = self.update_pos(self.pos,self.vel,self.update_rate)
            # print(self.pos)
            self.rate.sleep()

if __name__ == '__main__':
    tfrm = Tfbroadcaster()
    tfrm.init()
    tfrm.main()
    rospy.spin()
