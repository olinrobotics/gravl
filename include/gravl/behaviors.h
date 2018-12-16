// References https://answers.ros.org/question/201977/include-header-file-from-another-package-indigo/
#pragma once

#include <ros/ros.h>
#include <string>
#include <vector>
#include <geometry_msgs/Twist.h>

struct Behavior
{
  std::string name;
  int priority;
  int id;
  geometry_msgs::Twist message;
};

std::vector<Behavior> getBehaviors(ros::NodeHandle n);
bool compareBehaviorId(Behavior b1, Behavior b2);
