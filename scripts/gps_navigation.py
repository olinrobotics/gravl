#!/usr/bin/env python
from math import *


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

#values to be updated with callback
current_longitude  = 0
current_latitude   = 0
current_angle      = 10 #make sure this is in degrees
waypoint_longitude = -0.1
waypoint_latitude  = -1

#data to be entered/calibrated
max_turn_angle = 45
kp1 = 1 #calibrate for steering angle
kp2 = 0.1 #calibrate for steering velocity

def calculate_desired_angle(clat, clong, wlat, wlong):
    longitude_difference =  wlong - clong
    latitude_difference = wlat - clat
    desired_angle_rad = atan2(longitude_difference, latitude_difference)
    desired_angle_deg = desired_angle_rad * (180 / pi)
    return desired_angle_deg

def calculate_steering_error(current_angle, desired_angle):
    error =  desired_angle - current_angle
    return error

def calculate_steering_angle(error, kp1):
    steering_angle = None
    if error < -max_turn_angle:
        steering_angle = -max_turn_angle
    elif error > max_turn_angle:
        steering_angle = max_turn_angle
    else:
        steering_angle = kp1 * -error #if error is positive, want negative motion to compensate
    return steering_angle

def run():
    print "running..."
    desired_angle = calculate_desired_angle(current_latitude, current_longitude, waypoint_latitude, waypoint_longitude)
    print desired_angle
    steering_error = calculate_steering_error(current_angle, desired_angle)
    print steering_error
    steering_angle = calculate_steering_angle(steering_error, kp1)
    print steering_angle

if __name__=='__main__':
    run()
