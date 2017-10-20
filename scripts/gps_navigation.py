#!/usr/bin/env python
from math import *
import rospy
from std_msgs.msg import String
from gps_common import GPSFix
from publish_steering import publish_steering_angle

"""
To Do:
How do we insert the sense/think/act framework?
What are the message types for the Hemisphere and how do we process?
Verify math
figure out how importing works
make publishers work
make publishers work with ackermann_msgs
create degrees to radians conversions
create callback function
calibrate data
figure out why extra file is created
"""
class GPS_navigation_node:
    """
    """
    def __init__(self, waypoint_longitude, waypoint_latitude):
    """
    Initializes a GPS Navigation Node.
    """
        # Current latiude of the tractor (coordinate).
        self.current_longitude = 0
        # Current longitude of the tractor (coordinate).
        self.current_latitude = 0
        # Angle the car is currently facing (degrees).
        self.current_angle = 10
        # Longitude of the intended waypoint.
        self.waypoint_longitude = waypoint_longitude
        # Latitude of the intended waypoint.
        self.waypoint_latitude = waypoint_latiude
        # Set up the subscribers for both GPS modules. 
        # TODO Check what the message type should be
        rospy.Subscriber("RTK_GPS", GPSFix, rtk_callback)
        # TODO Figure out what this message type will be. String for now.
        rospy.Subscriber("Hemisphere_GPS", String, hemisphere_callback)
        #################CALIBRATION########################
        # The maximum angle the tractor can turn at once (degrees).
        self.max_turn_angle = 45
        # The minimum angle the tractor can turn at once (degrees).
        self.min_turn_angle = -max_turn_angle
        # Proportional constant for steering angle calculation
        kp1 = 0.1
        # Proportional constant for steering velocity calculation.
        kp2 =  0.1
        # Proportional constant for forwards velocity calculation.
        kp3 = 0.1
    
    def rtk_callback(self, msg):
        """
        """
        self.current_longitude = msg.longitude
        self.current_latitude = msg.latitude
        if (self.current_latitude != self.waypoint_latitude && \
            self.current_longitude != self.waypoint_longitude):
           deg_calculate_desired_angle()
        

    def hemisphere_callback():
        """
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

#########################CORE FUNCTIONS#########################
def deg_calculate_desired_angle(clat=self.current_latitude, clong=self.current_longitude, wlat=self.waypoint_latitude, wlong=self.waypoint_longitude):
    """
    Calculates the desired angle for the car based on the angle necessary
    to drive towards the waypoint.
    """
    longitude_difference =  wlong - clong
    latitude_difference = wlat - clat
    desired_angle_rad = atan2(longitude_difference, latitude_difference)
    desired_angle_deg = desired_angle_rad * (180 / pi)
    return desired_angle_deg #in degrees

def deg_calculate_steering_error(current_angle, desired_angle):
    """
    """
    error =  desired_angle - current_angle
    return error #in degrees

def deg_calculate_steering_angle(error, kp1):
    """
    """
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
