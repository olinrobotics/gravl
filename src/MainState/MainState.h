#ifndef MAIN_STATE_H
#define MAIN_STATE_H

#include <ros/ros.h>
#include <gravl/TwistLabeled.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <vector>

struct Behavior {
    const char* name = "safety";
    int priority = 0;
    gravl::TwistLabeled message = gravl::TwistLabeled();
};

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

  std::vector<Behavior> behavior_vector;
  bool is_activated;
  Behavior curr_state;

  void stateCB(const std_msgs::String& msg);
  void activateCB(const std_msgs::Bool& msg);
  void behaviorCB(const gravl::TwistLabeled& msg);
  void setState(const char* state);
  int checkBehavior(const char* name, int* priority);
  std::string getParamName(const char* name, int info);
};

#endif //MAIN_STATE_H
