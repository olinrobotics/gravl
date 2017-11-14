#ifndef STATE_H
#define STATE_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <ackermann_msgs/AckermannDrive.h>

class State{
public:
  explicit State();
private:
  ros::NodeHandle n;
  ros::Subscriber state;
  ros::Subscriber telesub;
  ros::Subscriber autosub;
  ros::Publisher drivepub;
  ackermann_msgs::AckermannDrive drive_msg;
  void stateCB(const std_msgs::Bool &msg);
  void teleCB(const ackermann_msgs::AckermannDrive &msg);
  void autoCB(const ackermann_msgs::AckermannDrive &msg);
};

#endif //STATE_H
