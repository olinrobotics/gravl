#!/usr/bin/env python
import roslib
import rospy
import tf

def update_pos(pos, vel,rate):
    pos[0] += vel[0]/rate
    pos[1] += vel[1]/rate
    return pos

if __name__ == '__main__':
    rospy.init_node('temp_tf_broadcaster')  #Initialized tf node

    # Tractor position,velocity in space
    pos = [0.0,0.0]
    vel = [1.0,0]
    update_rate = 10.0

    rate = rospy.Rate(update_rate) # tf publish rate

    while not rospy.is_shutdown():
        br = tf.TransformBroadcaster()
        br.sendTransform((pos[0], pos[1], 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "base_link",
                         "map")
        pos = update_pos(pos,vel,update_rate)
        print(pos)
        rate.sleep()
