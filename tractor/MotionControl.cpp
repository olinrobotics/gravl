#include "MotionControl.h"

void MotionControl::setup(ros::NodeHandle *nh){
	KANGAROO.begin(KANGAROO_SPEED); // Kangaroo com
	steering = new KangarooChannel(*k, '1');
	steering->start();
	gas = new KangarooChannel(*k, '2');
  	gas->start();

  	//drive_sub = new ros::Subscriber<ackermann_msgs::AckermannDrive>("/drive_cmd", &drive_cb);
}

void MotionControl::drive_cb(const ackermann_msgs::AckermannDrive &msg){

}