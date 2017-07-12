#ifndef MOTIONCONTROL_H
#define MOTIONCONTROL_H

#include "config.h"

#include "ros.h"
#include "ackermann_msgs/AckermannDrive.h"

class MotionControl{
public:
	static void setup(ros::NodeHandle *nh);
private:
	static void drive_cb(const ackermann_msgs::AckermannDrive &msg);
  static ros::Subscriber<ackermann_msgs::AckermannDrive> *driv;
};

#endif //MOTIONCONTROL_H
