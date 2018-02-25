#ifndef MOTION_H
#define MOTION_H

#include "RoboClaw.h"
#include "ros.h"
#include "geometry_msgs/Twist.h"
#include <gravl/RoboClawStats.h>

class Motion{
public:
	explicit Motion(ros::NodeHandle *nh, HardwareSerial *rc, const long baud, const uint8_t address);

private:
	RoboClaw *roboclaw;
	ros::Publisher *status;
	ros::Subscriber<geometry_msgs::Twist, Motion> *cmd;
	gravl::RoboClawStats stats;
	const uint8_t address;
	void stat_pub();
	void vel_callback(const geometry_msgs::Twist &message);
};

#endif //MOTION_H
