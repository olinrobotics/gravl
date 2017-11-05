#!/usr/bin/env python

from math import *
import rospy
from message_filters import TimeSynchronizer
from sensor_msgs.msg import NavSatFix
# TODO Ensure that this still works with the custom hemisphere message.
from Hemisphere.msg import Hemisphere
from ackermann_msgs.msg import AckermannDrive

class GPSNavigationNode:
    """
    ROS Node for GPS navigation using a rtk GPS and hemisphere vector GPS
    for an Ackermann-steered vehicle. Must be instantiated with a specific
    GPS waypoint list for the vehicle to drive towards in sequence.
    """
    def __init__(self, waypoint_array=[]):
        """
        Initializes a GPS Navigation Node.
        waypoint_array : An array of waypoints, in the format of 
                    [[lat, long], [lat, long], [lat, long]]
        """
        # Initial setup that can be reused for any number of waypoints.
        self.fix = rospy.Subscriber('fix', NavSatFix)
        self.dir = rospy.Subscriber('direction', Hemisphere)
        self.nav_data = \
                     message_filters.TimeSynchronizer([self.fix, self.dir], 10)
        self.pub = rospy.Publisher('cmds', AckermannDrive, queue_size = 10)
        # Current latiude of the tractor (coordinate).
        self.current_longitude = None
        # Current longitude of the tractor (coordinate).
        self.current_latitude = None
        # Angle the car is currently facing (degrees).
        self.current_angle = None
        # The target longitude of the tractor (coordinate)
        self.waypoint_longitude = None
        # the target latitude of the car.
        self.waypoint_latitude = None
        # The maximum angle the tractor can turn at once (degrees).
        self.max_turn_angle = 45
        # The minimum angle the tractor can turn at once (degrees).
        self.min_turn_angle = -max_turn_angle
        #Whether or not the waypoint has been reached.
        self.waypoint_reached = False
        # After synchronizing the data streams, we can register one callback.
        self.nav_data.registerCallback(self.update)

        # Set up for the PID controller.
        # Proportional constant for steering angle calculation
        self.kp1 = 0.1
        # Proportional constant for steering velocity calculation.
        self.kp2 =  0.1
        # Proportional constant for forwards velocity calculation.
        self.kp3 = 0.1

        # Iterate through each waypoint set.
        for waypoint in waypoint_array:
            while not self.waypoint_reached:
                self.waypoint_latiude = waypoint[0]
                self.waypoint_longitude = waypoint[1]
                rospy.spin()
            # At this point, waypoint_reached will be set to true by the
            # update function, so we'll need to set it to false to ensure
            # that it actually navigates to the next point.
            self.waypoint_reached = False

    def update(self):
        """
        Processes gps data and waypoint and publishes new steering angle.
        """
        # Double check there is data from fix and from the Hemisphere.
        if nav_data.longitude is None or nav_data.direction is None:
            return

        # Update class variables.
        self.current_longitude = nav_data.longitude
        self.current_latitude = nav_data.latitude
        self.current_angle = nav_data.direciton

        # Figure out how to proceed if the tractorhas not reached.
        if (isclose(self.current_latitude, self.waypoint_latitude) and \
            isclose(self.current_longitude, self.waypoint_longitude)):
            desired_angle = deg_calculate_desired_angle(self.current_latitude,\
                            self.current_longitude, self.waypoint_latitude, \
                            self.waypoint_longitude)
            current_error = deg_calculate_steering_error(self.current_angle, \
                            desired_angle)
            new_steering_angle = deg_calculate_steering_angle(current_error, \
                                 self.kp1)
            new_velocity = deg_calculate_forward_velocity(new_steering_angle, \
                           self.kp3)
            
            # Create and publish a drive message.
            drive_msg = AckermannDrive()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.steering_angle = new_steering_angle
            drive_msg.speed = abs(new_velocity)
            self.pub.publish(drive_msg)

        # Otherwise, we're ready to move on to the next waypoint.
        else: 
            self.waypoint_reached = True
            return

    #########################CORE FUNCTIONS#########################
    # TODO Decide if there's any reason to make this block static.

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
            return desired_angle - current_angle

    def deg_calculate_steering_angle(error, kp1):
        """
        Calculate the new steering angle for the vehicle.
        
        error : The difference between the current and desired angle.
        kp1 : Proportional constant for PID controller. Should be tuned 
        empirically.
        Returns angle (in degrees) that the vehicle should turn in.
        """
        steering_angle = None
        if error > abs(self.max_turn_angle):
            steering_angle = self.max_turn_angle
        else:
            steering_angle = kp1 * -error
        return steering_angle

    def deg_calculate_forward_velocity(angle, kp3):
        """
        Calculate the new forward velocity for the car based on the turn angle.
        
        angle : The new steering angle for the vehicle.
        kp3 : Proportional constant for the PID controller. Should be 
        tuned empircally.
        Returns the new forward velocity for the vehicle.
        """
        # TODO Confirm whether or not this math checks out. 
        forward_velocity = kp3 * \
                 (self.max_forward_speed/self.max_turn_angle) * angle
        if forward_velocity >= self.max_forward_speed:
            forward_velocity = self.max_forward_speed
        return forward_velocity


def run():
    rospy.init_node("GPSNavigationNode")
    try:
        # TODO Go through data set, pick waypoints (might be somewhat 
        # arbitrary but it would be good to test actual navigation.
        waypoint_array = []
        gps_nav_node = GPSNavigationNode(waypoint_array)
    except rospy.ROSInterruptException:
        pass    

if __name__=='__main__':
    run()
