#!/usr/bin/env python

from math import *
import rospy
import message_filters
# using string as a proxy for whatever the hemisphere outputs. Should be fixed.
from std_msgs.msg import String
from gps_common import GPSFix
from ackermann_msgs import AckermannDrive

"""
TODO:
How do we insert the sense/think/act framework?
What are the message types for the Hemisphere and how do we process?
Verify math
create degrees to radians conversions if necessary
decidee how to structure callback function more rigorously
calibrate data
figure out why extra file is created
"""
class GPS_navigation_node:
    """
    ROS Node for GPS navigation using a rtk GPS and hemisphere vector GPS
    for an Ackermann-steered vehicle. Must be instantiated with a specific
    GPS waypoint for the vehicle to drive towards.
    """
    def __init__(self, waypoint_longitude, waypoint_latitude):
    """
    Initializes a GPS Navigation Node.
    waypoint_longitude : Longitude coordinate of desired waypoint.
    waypoint_latitude : Latitude coordinate of desired waypoint.
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
        # The maximum angle the tractor can turn at once (degrees).
        self.max_turn_angle = 45
        # The minimum angle the tractor can turn at once (degrees).
        self.min_turn_angle = -max_turn_angle
        #################CALIBRATION########################
        # Proportional constant for steering angle calculation
        self.kp1 = 0.1
        # Proportional constant for steering velocity calculation.
        self.kp2 =  0.1
        # Proportional constant for forwards velocity calculation.
        self.kp3 = 0.1
        #Whether or not the waypoint has been reached.
        self.waypoint_reached = False
        # Set up the subscribers for both GPS modules. 
        while not waypoint_reached:
            # TODO Check what the message type should be
            rtk_sub = rospy.Subscriber("RTK_GPS", GPSFix, rtk_callback)
            # TODO Figure out what this message type will be. String for now.
            h_sub = rospy.Subscriber("Hemisphere_GPS", String, hemisphere_callback)
            data = message_filters.TimeSynchronizer([rtk_sub, ih_sub], 10)
            ts.registerCallback(callback)
            rospy.spin()
        print("Waypoint reached!")
    
    def callback(self, rtk_data, h_data):
        """
        Processes gps data and publishes new steering angle accordingly.
       
        rtk_data : Data from RTK GPS, time synchronized with hemisphere.
        h_data : Data from Hemisphere GPS, time synchronized with RTK. 
        """
        # Make sure we have both    
        if rtk_data is None or h_data is None:
            return
        # These message identifiers are a little arbitrary, need to fix.
        # TODO Figure out what is needed to access each field.
        self.current_longitude = rtk_data.longitude
        self.current_latitude = rtk_data.latitude
        self.current_angle = h_data.angle
        new_steering_angle = 0
        new_velocity = 0
        if (self.current_latitude != self.waypoint_latitude && \
            self.current_longitude != self.waypoint_longitude):
            desired_angle = deg_calculate_desired_angle(self.current_latitude,\
                            self.current_longitude, self.waypoint_latitude, \
                            self.waypoint_longitude)
            current_error = deg_calculate_steering_error(self.current_angle, \
                            desired_angle)
            new_steering_angle = deg_calculate_steering_angle(current_error, \
                                 self.kp1)
            new_velocity = deg_calculate_forward_velocity(new_steering_angle, \
                           self.kp3)
            drive_msg = AckermannDrive()
            drive_msg.steering_angle = new_steering_angle
            drive_msg.speed = abs(new_velocity)

        else: 
            self.waypoint_reached = True
            return

    #########################CORE FUNCTIONS#########################
    # TODO We should decide if these are class functions or static methods
    # we can use for wherever. I don't see a reason for making them static,
    # but I also don't know the whole situation.
    def deg_calculate_desired_angle(clat, clong, wlat, wlong):
        """
        Calculates the desired angle (in degrees) for the car based on the 
        angle necessary to drive towards the waypoint.
        clat : Current latitude coordinate of the vehicle.
        clong : Current longitude coordinate of the vehicle.
        wlat : Current latitude coordinate of the vehicle.
        wlong : Current longtiude coordinaate of the vehicle. 
        Returns the desired angle in degrees. 
        """
        longitude_difference =  wlong - clong
        latitude_difference = wlat - clat
        desired_angle_rad = atan2(longitude_difference, latitude_difference)
        desired_angle_deg = desired_angle_rad * (180 / pi)
        return desired_angle_deg

    def deg_calculate_steering_error(current_angle, desired_angle):
        """
        Calculates the difference between the current vehicle steering angle
        and the desired value.

        current_angle : The angle of the vehicle (likely based on sensor
        readings).
        desired_angle : The ideal angle of the vehicle (likely based on 
        calculation by means of a GPS waypoint). 
        Returns the number of degrees to turn to face the desired angle.
        """
        error =  desired_angle - current_angle
        return error

    def deg_calculate_steering_angle(error, kp1):
        """
        Calculate the new steering angle for the vehicle.
        
        error : The difference between the current and desired angle.
        kp1 : Proportional constant for PID controller. Should be tuned 
        empirically.
        Returns angle (in degrees) that the vehicle should turn in.
        """
        steering_angle = None
        if error < -max_turn_angle:
            steering_angle = -max_turn_angle
        elif error > max_turn_angle:
            steering_angle = max_turn_angle
        else:
            steering_angle = kp1 * -error #if error is positive, want negative motion to compensate
        return steering_angle

    def deg_calculate_forward_velocity(angle, kp3):
        """
        Calculate the new forward velocity for the car based on the turn angle.
        
        angle : The new steering angle for the vehicle.
        kp3 : Proportional constant for the PID controller. Should be 
        tuned empircally.
        Returns the new forward velocity for the vehicle.
        """
        forward_velocity = kp3 * (max_forward_speed/max_turn_angle) * angle
        # print  (max_forward_speed/max_turn_angle) * angle) 
        if forward_velocity >= max_forward_speed:
            forward_velocity = max_forward_speed
        return forward_velocity

# Testing functions.
def run():
    # Test with some coordinates.
    gps_test = GPSNavigatioNode(42.29, 72.26)
    print "running..."

if __name__=='__main__':
    run()
