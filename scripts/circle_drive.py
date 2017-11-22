#!/usr/bin/env python
import rospy                                                                    # ROS library for python use
from ackermann_msgs.msg import AckermannDriveStamped                            # ROS Ackermann Steering message

# Set Constants

# Set Variables
tractor_speed = 1
tractor_turn = 45

'''
Function: drives tractor along continuous circle
@arguments: wheel angle, drive speed
@returns: none
'''
def drive_circle(speed, turn):

    # Setup
    ack_msg = AckermannDriveStamped()
    rospy.init_node('circle_driver')
    drive_publish = rospy.Publisher('autodrive',AckermannDriveStamped, queue_size=1)
    r = rospy.Rate(20)

    # Command loop, runs while roscore is up
    while not rospy.is_shutdown():
        r.sleep()
        ack_msg.header.stamp = rospy.Time.now()
        ack_msg.drive.steering_angle = turn
        ack_msg.drive.speed = speed
        drive_publish.publish(ack_msg)

# Runs this section if the file is run in isolation
if __name__ == '__main__':

    try:
        drive_circle(tractor_speed, tractor_turn)
    except rospy.ROSInterruptException:
        pass
