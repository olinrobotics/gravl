#ifndef TELEOP_H
#define TELEOP_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <gravl/TwistLabeled.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <string.h>

class Teleop{
public:
  explicit Teleop();
private:
  ros::NodeHandle n;
  ros::Subscriber joystick_sub;
  ros::Publisher drivemsg_pub;
  ros::Publisher softestop_pub;
  ros::Publisher activate_pub;
  ros::Publisher state_pub;
  std_msgs::Bool stop_msg;
  std_msgs::Bool activate_msg;
  std_msgs::UInt8 state_msg;
  gravl::TwistLabeled drive_msg;

  void joyCB(const sensor_msgs::Joy::ConstPtr &joy);
  void softestop(bool stop);
  void activate(bool aut);
  std::string controllerType;
  bool estop;
  bool isActivated;
  int activateButton;
  int estopButton;
  bool estopButtonFlag;
  bool activateButtonFlag;

};

#endif //TELEOP_H
