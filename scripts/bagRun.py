#!/usr/bin/env python2
import rospy as rp
from std_msgs.msg import String
import subprocess
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
        rp.loginfo("collecting bag")
        subprocess.call(["roslaunch","gravl","bag_experiment-0.launch"])
        rp.loginfo("done bag")

if __name__ == '__main__':

    rp.init_node('bag_run')
    subCmd = rp.Subscriber('/user_input', String,userInputCB)
rp.spin()
