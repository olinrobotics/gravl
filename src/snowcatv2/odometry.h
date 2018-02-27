#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "ros.h"
#include <ros/time.h>
#include <tf/transform_broadcaster.h>

class Odometry{
public:
	explicit Odometry(ros::NodeHandle *nh);
	void odom_pub();

private:
	ros::NodeHandle *nh;
	geometry_msgs::TransformStamped *t;
	tf::TransformBroadcaster *broadcaster;
};

#endif //ODOMETRY_H
