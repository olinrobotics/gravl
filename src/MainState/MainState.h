#ifndef MAIN_STATE_H
#define MAIN_STATE_H

#include <ros/ros.h>
#include <gravl/TwistLabeled.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include "Behavior.h"
#include <vector>

class MainState{
public:
  explicit MainState();
private:
  ros::NodeHandle n;
  ros::Subscriber state_sub;
  ros::Subscriber activate_sub;
  ros::Subscriber behavior_sub;
  ros::Publisher state_pub;
  ros::Rate rate;
  std_msgs::UInt8 curr_state;

  std::vector<Behavior> behavior_vector;
  bool is_activated;

  void stateCB(const std_msgs::UInt8& msg);
  void activateCB(const std_msgs::Bool& msg);
  void behaviorCB(const gravl::TwistLabeled& msg);
  void setState(std_msgs::UInt8 state);
  void updateBehaviors();
  int getBehavior(int label, int* index);
};

#endif //MAIN_STATE_H
