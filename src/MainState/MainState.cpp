/*
 * @file MainState.cpp
 * @author Connor Novak
 * @date 2018-11-13
 *
 * Subscribes to /cmd_behavior, publishes msgs to /cmd_twist based on tractor's
 * current state.
 */

#include "MainState.h"
#include <ros/console.h>  // Used for logging
#include <iostream>

MainState::MainState()
 : rate(ros::Rate(2))
 , state_sub(n.subscribe("/state_controller/cmd_state", 1, &MainState::MainState::stateCB, this))
 , activate_sub(n.subscribe("/state_controller/cmd_activate", 1, &MainState::MainState::activateCB, this))
 , behavior_sub(n.subscribe("/state_controller/cmd_behavior", 10, &MainState::MainState::behaviorCB, this))
 , state_pub(n.advertise<std_msgs::String>("state", 1))
 , command_pub(n.advertise<geometry_msgs::Twist>("cmd_twist", 1)) {
   setActivation(false);
   behaviors = getBehaviors(n);
   setState("teleop");
}

void MainState::stateCB(const std_msgs::String& msg) {
  /*
   * @brief updates and publishes current state
   *
   * @param Can change curr_state via setState()
   */
    if (msg.data != curr_state.name) setState(msg.data.c_str());
}

void MainState::activateCB(const std_msgs::Bool& msg) {
  /*
   * @brief updates current activation state
   *
   * @param[out] can change is_activated via setActivation()
   */
  setActivation(msg.data);
}

void MainState::behaviorCB(const gravl::TwistLabeled& msg) {
/*
 * @brief pass on received twist if correct mode, handles priority mode change
 *
 * Checks incoming twist messages. If they match the current state, they are
 * published. If their priority is higher than that of the current state, the
 * state is changed.
 *
 * @param[out] can change curr_state
 * @param[out] updates curr_state message
 */

  if (is_activated) {

    // Get current behavior index
    int index;
    if (checkBehavior(msg.label.c_str(), &index)) {
        ROS_ERROR("Could not find behavior %s - is parameter space set up?", msg.label.c_str());
        return;
    }

    // priority 0 > priority 1 & 2, priority 1 > priority 2, priority 2 !> priority 2
    if (behaviors[index].priority == 0) setState(msg.label.c_str());
    else if (behaviors[index].priority == 1 && curr_state.priority != 0) setState(msg.label.c_str());

    // Update behavior, publish message
    if (msg.label.c_str() == curr_state.name) {
      curr_state.message = msg.twist;
      command_pub.publish(msg.twist);
    }

  } else ROS_INFO_THROTTLE(5, "Tractor not activated");
}

int MainState::setState(const char* state) {
  /*! \brief Updates state if different, publishes to state
  *
  * @param[in] name of state to activate
  * @param[out] updates curr_state attribute
  * @return 0 if updated, 1 if no state change, 2 if failed behavior check
  */

  if (curr_state.name != state) {

    int i;
    if (!checkBehavior(state, &i)) {
      curr_state = behaviors[i];
      ROS_INFO("Activating State %s", curr_state.name.c_str());
      std_msgs::String msg;
      msg.data = curr_state.name;
      state_pub.publish(msg);
    } else return 2;
  } else return 1;
  return 0;
}

int MainState::setActivation(bool activate) {
  /*
   * @brief Sets activation state to arg
   *
   * @param[in] activation state required
   * @param[out] updates attribute is_activated
   * @return 0 if updated, 1 otherwise
   */
   if (activate != is_activated) {
     if (activate) ROS_INFO("Activating Tractor");
     else ROS_INFO("Disactivating Tractor");
     return 0;
   } else return 1;
}

int MainState::checkBehavior(const char* name, int* index) {
/*
 *@brief get index of behavior in behaviors if exists
 *
 * @param[in]  behavior name
 * @param[in] pointer to index storage var
 * @param[out] variable pointed to by index
 * @return    return 1 if behavior d.n.e., 0 otherwise
 */
  for (int i = 0; i < behaviors.size(); i++) {
    if (behaviors[i].name == name) {
      *index = i;
      return 0;
    }
  }
  ROS_WARN("Behavior %s does not exist", name);
  return 1;
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "MainState");
  MainState mainstate;
  ros::spin();
}
