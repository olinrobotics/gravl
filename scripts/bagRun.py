#!/usr/bin/env python2
import rospy
# Set Variables
tractor_speed = 1
tractor_turn = 45

'''
Function: drives tractor along continuous circle
@arguments: wheel angle, drive speed
@returns: none
'''


def userInputCB(msg):
    """Start bag file """
    if(msg.data == "b"):
        subprocess.call("gravl","bag_experiment-zero.launch")


if __name__ == '__main__':

    rospy.init_node('bag_run')
    subCmd = rp.Subscriber('/user_input', String)
rp.spin()
