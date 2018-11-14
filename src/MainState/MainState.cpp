/*
 * @file MainState.cpp
 * @author Connor Novak
 * @date 2018-11-13
 *
 * Subscribes to /cmd_behavior, publishes msgs to /cmd_vel based on tractor's
 * current state.
 */

#include "MainState.h"

MainState::MainState() {

  // State Handling
  state_sub = n.subscribe("joy_state", 1, &MainState::MainState::stateCB, this);
  activate_sub = n.subscribe("joy_active", 1, &MainState::MainState::activateCB, this);
  behavior_sub = n.subscribe("cmd_behavior", 10, &MainState::MainState::behaviorCB, this);
  state_pub = n.advertise<std_msgs::UInt8>("curr_state", 1);
  curr_state = 1;
  is_activated = false;

}

void MainState::stateCB(const std_msgs::UInt8& msg) {
  // Callback for joy_state, updates and publishes curr_state
    curr_state = msg.data;
    ROS_INFO("Activating State %i", curr_state);
    state_pub.publish(msg);
}

void MainState::activateCB(const std_msgs::Bool& msg) {
  // Callback for joy_active, updates activated state
  is_activated = msg.data;
  if (is_activated) {ROS_INFO("Activating Tractor");}
  else {ROS_INFO("Disactivating Tractor");}
}

void MainState::behaviorCB(const gravl::TwistLabeled& msg) {
  ROS_INFO("Message from node %i", msg.label);
}
int main(int argc, char** argv) {

  ros::init(argc, argv, "MainState");
  MainState mainstate;
  ros::spin();
}
