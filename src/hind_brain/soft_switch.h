/******************************************************************************
 * @file        soft_switch.h
 * Software  header for OAK soft_switch (Olin Autonomous Kore)
 * @author      Carl Moser
 * @email       carl.moser@students.olin.edu
 * @version     1.0
 * @date        24/07/17
 ******************************************************************************/

#ifndef OAK_SOFT_SWITCH_H
#define OAK_SOFT_SWITCH_H

 #include "ros.h"
 #include "std_msgs/Bool.h"

class OAKSoftSwitch{
public:
  explicit OAKSoftSwitch(ros::NodeHandle *nh, const char* name, const int pin);

private:
  ros::Subscriber<std_msgs::Bool, OAKSoftSwitch> *signalIn;
  const int pin;
  void softCB(const std_msgs::Bool &sig);
};

#endif //OAK_SOFT_SWITCH_H
