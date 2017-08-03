#include "soft_switch.h"

/******************************************************************************
 * @file        soft_switch.cpp
 * Software  class for OAK (Olin Autonomous Kore)
 * @author      Carl Moser
 * @email       carl.moser@students.olin.edu
 * @version     1.0
 * @date        24/07/17
 ******************************************************************************/


OAKSoftSwitch::OAKSoftSwitch(ros::NodeHandle *nh, const char* name, const int pin):pin(pin){
  signalIn = new ros::Subscriber<std_msgs::Bool, OAKSoftSwitch>(name, &OAKSoftSwitch::softCB, this);
  nh->subscribe(*signalIn);
  pinMode(pin, OUTPUT);
}

void OAKSoftSwitch::softCB(const std_msgs::Bool &sig){
  digitalWrite(pin, sig.data);
}
