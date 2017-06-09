#ifndef MOTIONCONTROL_H
#define MOTIONCONTROL_H

#include "config.h"

#include "ros.h"
#include "ackermann_msgs/AckermannDrive.h"
#include <Kangaroo.h>

class MotionControl{
public:
	void setup(ros::NodeHandle *nh);
	void drive_cb(const ackermann_msgs::AckermannDrive &msg);
private:
	ros::Subscriber<ackermann_msgs::AckermannDrive> *drive_sub;

	KangarooSerial *k;
	KangarooChannel *steering;
	KangarooChannel *gas;
};
	
#endif //MOTIONCONTROL_H)