#ifndef DRIVER_H
#define DRIVER_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <gravl/Hemisphere.h>
#include <sensor_msgs/NavSatFix.h>


class Driver{
public:
	explicit Driver();
private:
	tf::TransformBroadcaster odomBroadcaster;
	tf::Transform odomTransform;
};

#endif //DRIVER_H