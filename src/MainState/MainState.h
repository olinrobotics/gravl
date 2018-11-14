#ifndef MAIN_STATE_H
#define MAIN_STATE_H

#include <ros/ros.h>
#include <gravl/TwistLabeled.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>

class MainState{
public:
  explicit MainState();

private:
  ros::NodeHandle n;
  ros::Subscriber state_sub;
  ros::Subscriber activate_sub;
  ros::Subscriber behavior_sub;
  ros::Publisher state_pub;
  int curr_state;
  bool is_activated;

  // Callback functions
  void stateCB(const std_msgs::UInt8& msg);
  void activateCB(const std_msgs::Bool& msg);
  void behaviorCB(const gravl::TwistLabeled& msg);

  // Getters and Setters
  //TODO:
};

#endif //MAIN_STATE_H
