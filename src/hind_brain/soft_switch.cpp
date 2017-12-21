#include "soft_switch.h"

/******************************************************************************
 * @file        soft_switch.cpp
 * Software  class for OAK (Olin Autonomous Kore)
 * @author      Carl Moser
 * @email       carl.moser@students.olin.edu
 * @version     1.0
 * @date        24/07/17
 ******************************************************************************/

/*
 * Constructor for the class
 *
 * Initializes a publisher and subsciber
 * attaches the estop pin
 */
OAKSoftSwitch::OAKSoftSwitch(ros::NodeHandle *nh, const char* name, const int pin):pin(pin){
  signalIn = new ros::Subscriber<std_msgs::Bool, OAKSoftSwitch>(name, &OAKSoftSwitch::softCB, this);
  nh->subscribe(*signalIn);
  pinMode(pin, OUTPUT);
}

/*
 * Function that runs if a message is sent to the subscriber
 * and toggles the pin accordingly
 */
void OAKSoftSwitch::softCB(const std_msgs::Bool &sig){
  digitalWrite(pin, sig.data);
}
