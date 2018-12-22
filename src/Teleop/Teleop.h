/*
 * @file Teleop.h
 * @brief function prototypes for teleop node
 *
 * @author Carl Moser
 * @email carl.moser@students.olin.edu
 */

#ifndef TELEOP_H
#define TELEOP_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <gravl/TwistLabeled.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <string.h>
#include <gravl/behaviors.h>

class Teleop{
public:
  explicit Teleop();
private:
  ros::NodeHandle n;
  ros::Subscriber joystick_sub;
  ros::Publisher activate_pub;
  ros::Publisher drivemsg_pub;
  ros::Publisher softestop_pub;
  ros::Publisher state_pub;
  std_msgs::Bool stop_msg;
  std_msgs::Bool activate_msg;
  Behavior state_behavior;
  gravl::TwistLabeled drive_msg;
  std::vector<Behavior> behaviors;

  void joyCB(const sensor_msgs::Joy::ConstPtr &joy);
  void softestop(bool stop);
  void activate(bool aut);
  void state(Behavior behavior);
  int incrementState(float dir);
  std::string controllerType;
  bool estop;
  bool isActivated;
  int activateButton;
  int estopButton;
  int behaviorAxis;
  bool estopButtonFlag;
  bool activateButtonFlag;
  bool behaviorAxisFlag;

};

#endif //TELEOP_H
