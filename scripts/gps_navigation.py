#!/usr/bin/env python
from math import *
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
#from ackermann_msgs import AckermannDrive
"""
Notes:
ackermann msgs still not working
"""



# GPS navigation pseudocode.

# GPS
# Take in an array of waypoints
# Subscribe to topics for RTK GPS and Hemisphere GPS
# Use timesynchronizer to connect RTK and Hemisphere data.
# Run callback function with next waypoint.
# Rospy.spin()

# Callback function
# Take in waypoint [latitude longitude],current location [latitude longitude]
# and current angle [assuming degrees for now, for now, assume the gps is
# facing perpendicular to the car to the left so that car turns on 0-180.
# First, calculate desired angle to move. Take tangent of difference in
# longitude over difference in latitude.
# Calculate current error by subtracting the desired - existing angle
# Multiply error by kp1 to get new steering angle
# Multiply error by kp2 to get new velocity
# Tune the p values to be accurate

#UPDATE WITH CALLBACK###########################################################
current_longitude  = 0
current_latitude   = 0
current_x          = 0
current_y          = 0
current_angle      = atan2(current_y, current_x) * (180/pi) #must be in degrees
waypoint_longitude = -1
waypoint_latitude  = -1
pub                = None
#CALIBRATION####################################################################
max_turn_angle = 45               #in degrees
min_turn_angle = -max_turn_angle  #since zero degrees is straight, max turns
                                  #right and min turns left

kp1 = 1 #calibrate for steering angle
kp2 = 1 #calibrate for steering velocity
kp3 = 1 #calibrate for forwards velocity

max_forward_speed = 2 #speed goes from 0 to 2
min_forward_speed = 0.25 #for calculations
max_turn_speed = 0.1 #calibrate to something reasonable (rad/s)

#CORE FUNCTIONS#################################################################
def deg_calculate_desired_angle(clat, clong, wlat, wlong):
    """
    Calculates desired angle based on difference between current latitude and
    longitude and the set waypoint latitude and longitude
    clat = current latitude
    clong = current longitude
    wlat = waypoint latitude
    wlong = waypoint longitude

    Outputs in degrees
    """
    longitude_difference =  wlong - clong
    latitude_difference = wlat - clat
    desired_angle_rad = atan2(longitude_difference, latitude_difference)
    desired_angle_deg = desired_angle_rad * (180 / pi)
    return desired_angle_deg #in degrees

def deg_calculate_steering_error(current_angle, desired_angle):
    error =  desired_angle - current_angle
    return error #in degrees

def deg_calculate_steering_angle(error, kp1):
    """
    Takes error and accounts for it with steering angle. kp1 changes with
    steering angle. kp1 changes how "direct" steering is-higher kp1 results in
    sharper turning.

    Outputs in degrees
    """
    steering_angle = None
    if error < -max_turn_angle:
        steering_angle = -max_turn_angle
    elif error > max_turn_angle:
        steering_angle = max_turn_angle
    else:
        steering_angle = kp1 * -error   #if error is positive, want negative
    return steering_angle #in degrees   #motion to compensate

def deg_calculate_forward_velocity(angle, kp3):
    """
    Establishes linear relationship between angle and speed
    more angle = less speed, can be adjusted with kp3 value
    """
    slope = (max_forward_speed-min_forward_speed)/max_turn_angle
    forward_velocity = -kp3 * slope * abs(angle) + max_forward_speed

#SUBSCRIBERS####################################################################
def callback_fix(data):
    global current_longitude
    global current_latitude
    current_longitude = data.longitude
    current_latitude = data.latitude
    update()

def callback_vel(data):
    global current_x
    global current_y
    current_x = data.twist.linear.x
    current_y = data.twist.linear.y
    update()

def run():
    rospy.init_node('GPS_Nav', anonymous=True)
    rospy.Subscriber('fix', NavSatFix, callback_fix)
    rospy.Subscriber('vel', TwistStamped, callback_vel)
    global pub
    pub = rospy.Publisher('cmds', Float32, queue_size=10)
    rospy.spin()

#CALCULATED VALUES##############################################################
desired_angle = deg_calculate_desired_angle(current_latitude, current_longitude, waypoint_latitude, waypoint_longitude)
steering_error = deg_calculate_steering_error(current_angle, desired_angle)
steering_angle = deg_calculate_steering_angle(steering_error, kp1)
forward_velocity = deg_calculate_forward_velocity(steering_angle, kp3)

def update():
    print "running..."      #print statements for vertification
    print desired_angle
    print steering_error
    print steering_angle
    print forward_velocity
    print current_longitude
    print current_latitude
    print current_x
    print current_y
    pub.publish(steering_angle)
    pub.publish(forward_velocity)
if __name__=='__main__':
    run()
