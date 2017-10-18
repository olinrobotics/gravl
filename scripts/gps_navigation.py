#!/usr/bin/env python
from math import *
import rospy
from std_msgs.msg import String

"""
To Do:
Verify math
figure out how importing works
make publishers work
make publishers work with ackermann_msgs
create degrees to radians conversions
create callback function
calibrate data
figure out why extra file is created
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

#########################UPDATE WITH CALLBACK#########################
current_longitude  = 0
current_latitude   = 0
current_angle      = 10 #make sure this is in degrees
waypoint_longitude = -0.1
waypoint_latitude  = -1

#########################CALIBRATION#########################
max_turn_angle = 45               #in degrees
min_turn_angle = -max_turn_angle

kp1 = 1 #calibrate for steering angle
kp2 = 1 #calibrate for steering velocity
kp3 = 1 #calibrate for forwards velocity

max_forward_speed = 2 #speed goes from 0 to 2
min_forward_speed = 0
max_turn_speed = 0.1 #calibrate to something reasonable (rad/s)

#########################CORE FUNCTIONS#########################
def deg_calculate_desired_angle(clat, clong, wlat, wlong):
    longitude_difference =  wlong - clong
    latitude_difference = wlat - clat
    desired_angle_rad = atan2(longitude_difference, latitude_difference)
    desired_angle_deg = desired_angle_rad * (180 / pi)
    return desired_angle_deg #in degrees

def deg_calculate_steering_error(current_angle, desired_angle):
    error =  desired_angle - current_angle
    return error #in degrees

def deg_calculate_steering_angle(error, kp1):
    steering_angle = None
    if error < -max_turn_angle:
        steering_angle = -max_turn_angle
    elif error > max_turn_angle:
        steering_angle = max_turn_angle
    else:
        steering_angle = kp1 * -error #if error is positive, want negative motion to compensate
    return steering_angle #in degrees

def deg_calculate_forward_velocity(angle, kp3):
    forward_velocity = kp3 * (max_forward_speed/max_turn_angle) * angle
    print  (max_forward_speed/max_turn_angle) * angle  #something here is wrong with datatypes
    if forward_velocity >= max_forward_speed:
        forward_velocity = max_forward_speed
    return forward_velocity

#########################CALCULATED VALUES#########################
desired_angle = deg_calculate_desired_angle(current_latitude, current_longitude, waypoint_latitude, waypoint_longitude)
steering_error = deg_calculate_steering_error(current_angle, desired_angle)
steering_angle = deg_calculate_steering_angle(steering_error, kp1)
forward_velocity = deg_calculate_forward_velocity(steering_angle, kp3)

def run():
    print "running..."
    print desired_angle
    print steering_error
    print steering_angle
    print forward_velocity

if __name__=='__main__':
    run()
