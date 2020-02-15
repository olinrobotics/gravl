#include "soft_switch.h"

/*
  @file         soft_switch.cpp
  Software class for OAK soft_switch (Olin Autonomous Kore)
  @author       Carl Moser
  @maintainer   Olin GRAVL
  @email        olingravl@gmail.com
  @version      1.1
  @date         2020-02-14
*/


OAKSoftSwitch::OAKSoftSwitch(ros::NodeHandle *nh, const char* name, const int pin):pin(pin){
  // Constructor - init subscriber & attach pin.
  signalIn = new ros::Subscriber<std_msgs::Bool, OAKSoftSwitch>(name, &OAKSoftSwitch::softCB, this);
  nh->subscribe(*signalIn);
  pinMode(pin, OUTPUT);
}


void OAKSoftSwitch::softCB(const std_msgs::Bool &sig){
  // Toggle estop pin to received state.
  digitalWrite(pin, sig.data);
}
