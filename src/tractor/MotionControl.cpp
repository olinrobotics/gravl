#include "MotionControl.h"

ros::Subscriber<ackermann_msgs::AckermannDrive> *MotionControl::driv;

void MotionControl::setup(ros::NodeHandle *nh){
   driv = new ros::Subscriber<ackermann_msgs::AckermannDrive>("/drive_cmd", &drive_cb);
   nh->subscribe(*driv);
}

void MotionControl::drive_cb(const ackermann_msgs::AckermannDrive &msg){

}
