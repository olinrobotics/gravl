/*
  @file       soft_switch.h
  Software header for OAK soft_switch (Olin Autonomous Kore)
  @author     Carl Moser
  @maintainer Olin GRAVL
  @email      olingravl@gmail.com
  @version    1.1
  @date       2020-02-14
*/

#ifndef OAK_SOFT_SWITCH_H
#define OAK_SOFT_SWITCH_H

#include "ros.h"
#include "std_msgs/Bool.h"

/*
  Class that connects a digital pin to a boolean rostopic.

  Usage:
    Instantiate class with nodehandle, topic name, & hardware pin number.
*/
class OAKSoftSwitch {

public:
  explicit OAKSoftSwitch(ros::NodeHandle *nh, const char* name, const int pin);

private:
  ros::Subscriber<std_msgs::Bool, OAKSoftSwitch> *signalIn;
  const int pin;
  void softCB(const std_msgs::Bool &sig);

};

#endif
