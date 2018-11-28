/*
 * @file MainState.cpp
 * @author Connor Novak
 * @date 2018-11-13
 *
 * Subscribes to /cmd_behavior, publishes msgs to /cmd_vel based on tractor's
 * current state.
 */

#include "MainState.h"

MainState::MainState()
 : rate(ros::Rate(2))
 , state_sub(n.subscribe("joy_state", 1, &MainState::MainState::stateCB, this))
 , activate_sub(n.subscribe("joy_active", 1, &MainState::MainState::activateCB, this))
 , behavior_sub(n.subscribe("cmd_behavior", 10, &MainState::MainState::behaviorCB, this))
 , state_pub(n.advertise<std_msgs::UInt8>("curr_state", 1))
 , curr_state()
 , is_activated(false)
{
 curr_state.data = 1;
 updateBehaviors();
}

void MainState::stateCB(const std_msgs::UInt8& msg) {
  // Callback for joy_state, updates and publishes curr_state
    if (msg.data != curr_state.data) {setState(msg);}
}

void MainState::activateCB(const std_msgs::Bool& msg) {
  // Callback for joy_active, updates activated state
  is_activated = msg.data;
  if (is_activated) {ROS_INFO("Activating Tractor");}
  else {ROS_INFO("Disactivating Tractor");}
}

void MainState::behaviorCB(const gravl::TwistLabeled& msg) {
/*
  if (is_activated) {
    if (msg.header.stamp != behavior_list[msg.label].header.stamp) {
      return;
    }
  }*/

  ROS_INFO("Message from node %i", msg.label);
}

void MainState::setState(std_msgs::UInt8 state) {
  /*! \brief Updates state with new state.
  *
  * setState updates the current state with a given state, publishes an info
  * message, and publishes the new state to the topic /curr_state
  */

  curr_state = state;
  ROS_INFO("Activating State %i", curr_state.data);
  state_pub.publish(state);
}

void MainState::updateBehaviors() {
  /*! \brief Updates behavior parameters
  *
  * updateBehaviors checks the behavior namespace on the parameter server and
  * populates behavior_list with the behaviors listed there.
  */
  Behavior temp_behavior(2);
  temp_behavior.getId();
}

/*  int i = 0;
  std::map<std::string, std::string> temp_list;

  if(n.getParam("/behaviors", temp_list)) {
    std::map<std::string, std::string>::iterator iterator = temp_list.begin();
    while (iterator != temp_list.end()){
      std::cout << iterator->first << "::" << iterator->second << std::endl;
      iterator++;
    }
  }
} */


int main(int argc, char** argv) {

  ros::init(argc, argv, "MainState");
  MainState mainstate;
  ros::spin();
}
