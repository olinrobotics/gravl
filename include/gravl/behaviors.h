#include <ros/ros.h>
#include <string>
#include <vector>

struct Behavior
{
  std::string name;
  int priority;
  int id;
};

std::vector<Behavior> getBehaviors(ros::NodeHandle n);
