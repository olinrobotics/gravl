#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "ros.h"
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <Encoder.h>

class Odometry{
public:
	explicit Odometry(ros::NodeHandle *nh);
	void odomPub();

private:
	ros::NodeHandle *nh;
	geometry_msgs::TransformStamped *t;
	tf::TransformBroadcaster *broadcaster;
	Encoder *left_wheel;
	Encoder *right_wheel;
	long left_position;
	long right_position;
};

#endif //ODOMETRY_H
