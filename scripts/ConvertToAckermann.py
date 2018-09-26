"""
Subscribes to a twist message and converts it to an ackermann message.
Subscribes to /twist_vel
Publishes to /cmd_vel

Assumes all inputs are normalized between -1 and 1

Last Edited: 9/26/18
Author: Amy Phung
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDrive
from math import pi


def callback(data, callback_args):
    """
    Callback function for subscriber - Converts message + publishes to /cmd_vel

    Args:
        data - the message picked up by the subscriber - should be a twist message
        callback_args - list of arguments for function - typically includes
            publisher object, can be expanded in future to include parameters
            (ex: max steering velocity, max acceleration, etc.)
    """
    # Unpack variables from callback_args - callback args is a list for future support of more parmeters
    pub = callback_args[0]

    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    [linear_vel, steering_angle] = convertToAckermann(data.linear.x, data.angular.z)
    publish(pub, linear_vel, steering_angle)

def init_subscriber(node, topic, msg_type, callback, callback_args):
    """
    Initializes subscriber

    Args:
        node - name of node
        topic - name of topic to subscribe to
        msg_type - message type
        callback - function to call when new message is recieved
        callback_args - list of arguments for function - typically includes
            publisher object, can be expanded in future to include parameters
            (ex: max steering velocity, max acceleration, etc.)
    """
    rospy.init_node(node, anonymous=True) # TODO: What to call this?
    rospy.Subscriber(topic, msg_type, callback, callback_args)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def init_publisher(node, topic, msg_type):
    """
    Initializes publisher

    Args:
        node - name of node
        topic - name of topic to publish to
        msg_type - message type

    Returns:
        pub - Publisher object
    """
    pub = rospy.Publisher(topic, msg_type, queue_size=10)
    rospy.init_node(node, anonymous=True)
    rate = rospy.Rate(10) # 10hz
    return pub

def convertToAckermann(linear_vel, angular_vel):
    """
    Converts linear and angular velocities to linear velocity and steering angle for
    ackermann messages

    Args:
        linear_vel - forward linear velocity from Twist message (should be between -1 and 1)
        angular_vel - angular velocity from Twist message (should be between -1 and 1)
    """

    max_steer_vel = 1 # Sets it to max for now - can be changed later
    min_steer_vel = 0.1 # Change depending on how fast tractor needs to move while steering
    max_angle = 1 # Everything is normalized between -1 and 1

    if angular_vel > max_steer_vel:
        angular_vel = max_steer_vel
    elif angular_vel < -max_steer_vel:
        angular_vel = -max_steer_vel

    steering_angle = ((max_steer_vel - angular_vel)/max_steer_vel) * max_angle

    if abs(steering_angle) > 0 and linear_vel == 0:
        linear_vel = min_steer_vel
    return [linear_vel, steering_angle]

def publish(pub, linear_vel, steering_angle): # AckermannDrive Example: https://answers.ros.org/question/265660/how-to-create-ackermanndrivestamped-messages-in-python/
    """
    Publishes ackermann message

    Args:
        pub - publisher
        linear_vel - linear velocity in (should be between -1 and 1)
        steering_angle - steering angle in radians (should be between -1 and 1)
    """
    ack_msg = AckermannDrive()
    ack_msg.speed = linear_vel
    ack_msg.steering_angle = steering_angle
    pub.publish(ack_msg)


if __name__ == '__main__':
    try:
        pub = init_publisher('ConvertToAckermann',"/cmd_vel",AckermannDrive) # TODO: Rename this and node

        callback_args = [pub]
        # TODO: subscribe to correct twist message source
        init_subscriber('ConvertToAckermann',"/twist_vel",Twist,callback,callback_args)

    except rospy.ROSInterruptException:
        pass
