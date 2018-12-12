/*
 * @file MainState.cpp
 * @author Connor Novak
 * @date 2018-11-13
 *
 * Subscribes to /cmd_behavior, publishes msgs to /cmd_vel based on tractor's
 * current state.
 */

#include "MainState.h"
#include <ros/console.h>  // Used for logging
#include <geometry_msgs/Twist.h>
#include <vector>

MainState::MainState()
 : rate(ros::Rate(2))
 , state_sub(n.subscribe("/state_controller/cmd_state", 1, &MainState::MainState::stateCB, this))
 , activate_sub(n.subscribe("/state_controller/cmd_activate", 1, &MainState::MainState::activateCB, this))
 , behavior_sub(n.subscribe("/state_controller/cmd_behavior", 10, &MainState::MainState::behaviorCB, this))
 , state_pub(n.advertise<std_msgs::String>("state", 1))
 , command_pub(n.advertise<geometry_msgs::Twist>("cmd_twist", 1))
 , curr_state()
 , is_activated(false) {
   setState("teleop");
}

void MainState::stateCB(const std_msgs::String& msg) {
  // Callback for joy_state, updates and publishes curr_state
    if (msg.data != curr_state.name) setState(msg.data.c_str());
}

void MainState::activateCB(const std_msgs::Bool& msg) {
  // Callback for joy_active, updates activated state
  is_activated = msg.data;
  if (is_activated) ROS_INFO("Activating Tractor");
  else {ROS_INFO("Disactivating Tractor");}
}

void MainState::behaviorCB(const gravl::TwistLabeled& msg) {
/*
 * @brief pass on received twist if correct mode, handles priority mode change
 *
 * Checks incoming twist messages. If they match the current state, they are
 * published. If their priority is higher than that of the current state, the
 * state is changed.
 */

  if (is_activated) {

    // Get current behavior priority
    int priority;
    if (checkBehavior(msg.label.c_str(), &priority)) {
        ROS_ERROR("Could not find behavior - is parameter space set up?");
        return;
    }

    // Estop overwrites Teleop overwrites Bn
    if (priority == 0) setState(msg.label.c_str());
    else if (priority == 1 && curr_state.priority != 0) setState(msg.label.c_str());

    // Update behavior, publish message
    curr_state.message = msg;
    if (msg.label.c_str() == curr_state.name) {
      command_pub.publish(msg.twist);
    }

  } else ROS_INFO_THROTTLE(5, "Tractor not activated");
}

void MainState::setState(const char* state) {
  /*! \brief Updates state with new state.
  *
  * setState updates the current state with a given state, publishes an info
  * message, and publishes the new state to the topic /curr_state
  */

  if (curr_state.name != state) {

    int priority;
    if (checkBehavior(state, &curr_state.priority)) {
      curr_state.name = state;
      ROS_INFO("Activating State %s", curr_state.name);
      std_msgs::String msg;
      msg.data = curr_state.name;
      state_pub.publish(msg);
    }
  }
}

int MainState::checkBehavior(const char* name, int* priority) {
/*
 *@brief get priority of given behavior; error if d.n.e.
 *
 * @param[in]  behavior name
 * @param[in]  pointer to priority storage variable
 * @param[out] priority, if behavior exists
 * return      integer error code
 */
  std::string full_name = getParamName(name, 0);
  if (n.getParam(full_name, *priority)) return 0;
  else return 1;
}

std::string MainState::getParamName(const char* name, int info) {
  /*
   * @brief: Returns global ns name for info from named behavior
   *
   * TODO: Failure cases pretty shitty currently
   * @param[in] name of behavior
   * @param[in] info to get (1 for priority, 0 for id)
   * @return string of name
   */

   std::string full_name("/behaviors/");
   full_name.append(name);
   if (info == 1)      full_name.append("/priority");
   else if (info == 0) full_name.append("/id");

   return full_name;
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "MainState");
  MainState mainstate;
  ros::spin();
}
