#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <ros/ros.h>
#include "gravl/TwistLabeled.h"

class Behavior {
public:
  explicit Behavior(int id);
  bool checkTwist(gravl::TwistLabeled);

// Getters and Setters
  gravl::TwistLabeled getMessage();

private:
  int id;
  char name;
  gravl::TwistLabeled message;

};

#endif //BEHAVIOR_H
