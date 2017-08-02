#ifndef TELEOP_H
#define TELEOP_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <std_msgs/Bool.h>

class Teleop{
public:
  explicit Teleop();
private:
  ros::NodeHandle n;
  ros::Subscriber joystick;
  ros::Publisher drive;
  ros::Publisher softestop;
  std_msgs::Bool stop_msg;
  ackermann_msgs::AckermannDrive drive_msg;

  void joyCB(const sensor_msgs::Joy::ConstPtr &joy);

};

#endif //TELEOP_H
