#ifndef MAIN_STATE_H
#define MAIN_STATE_H

#include <ros/ros.h>
#include <gravl/TwistLabeled.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <vector>
#include <gravl/behaviors.h>

class MainState{
public:
  explicit MainState();
private:
  ros::NodeHandle n;
  ros::Subscriber state_sub;
  ros::Subscriber activate_sub;
  ros::Subscriber behavior_sub;
  ros::Publisher state_pub;
  ros::Publisher command_pub;
  ros::Rate rate;

  std::vector<Behavior> behaviors;
  Behavior curr_state;
  bool is_activated;

  void stateCB(const std_msgs::String& msg);
  void activateCB(const std_msgs::Bool& msg);
  void behaviorCB(const gravl::TwistLabeled& msg);
  int setState(const char* state);
  int setActivation(bool activate);
  int checkBehavior(const char* name, int* index);
};

#endif //MAIN_STATE_H
