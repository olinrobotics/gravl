#ifndef MOTION_H
#define MOTION_H

#include "RoboClaw.h"
#include "ros.h"
#include "geometry_msgs/Twist.h"
#include <gravl/RoboClawStats.h>

#define QPPS_PER_REV 300
#define REV_PER_METER 120 //Placeholder

class Motion{
public:
	explicit Motion(ros::NodeHandle *nh, HardwareSerial *rc, const long baud, const uint8_t address);

private:
	RoboClaw *roboclaw;
	ros::Publisher *status;
	ros::Subscriber<geometry_msgs::Twist, Motion> *cmd;
	gravl::RoboClawStats stats;
	const uint8_t address;
	int32_t lspeed;
	int32_t rspeed;
	void stat_pub();
	void vel_callback(const geometry_msgs::Twist &vel);
};

#endif //MOTION_H
