/*
 * @file Teleop.cpp
 * @brief Node for sending joystick commands to state controller
 *
 * This contains functions for parsing raw joystick output based on
 * the class of joystick as set in the rosparam controllerType and
 * publishing parsed output to the main state controller
 * @author Carl Moser
 * @email carl.moser@students.olin.edu
 * @maintainer Kubo
 * @email olingravl@gmail.com
 * @version 1.1.0
 *
 * Takes input from controller specified by controllerType param, outputs to
 * main state controller - handles movement messages and state change commands
 *
 * @author Carl Moser
 * @email carl.moser@students.olin.edu
 */

#include "Teleop.h"

/*
 * @brief Advertises & subscribes topics, loads joystick settings
 *
 * This constructor has two default args that specify the gamepad -
 * One explicitly when loading the rosparam, another implicitly when
 * initializing the button and axis attributes.
 * TODO(connor@students): Error for improper pass of controller, remove
 * double-default
 * TODO(connor@students): Handle namespaces better
 */
Teleop::Teleop()
: n("~")
, joystick_sub(n.subscribe("/joy", 10, &Teleop::joyCB, this))
, activate_pub(n.advertise<std_msgs::Bool>("/state_controller/cmd_activate", 1))
, drivemsg_pub(n.advertise<gravl::TwistLabeled>("/state_controller/cmd_behavior", 1))
, softestop_pub(n.advertise<std_msgs::Bool>("/softestop", 1))
, state_pub(n.advertise<std_msgs::UInt8>("/state_controller/cmd_state", 1))
, estop(false)
, isActivated(false)
, activateButton(0)
, estopButton(1)
, behaviorAxis(7)
, estopButtonFlag(false)
, activateButtonFlag(false)
, behaviorAxisFlag(false)
{
  drive_msg.label = 1; // TODO(connor@students): Actually get label
  n.param<std::string>("controllerType", controllerType, "gamepad");
  if (controllerType == "gamepad"){
    activateButton = 0;
    estopButton = 1;
    behaviorAxis = 7;
  }
  if (controllerType == "joystick"){
    activateButton = 6;
    estopButton = 0;
    behaviorAxis = 7;
  }
}

/*
 * @brief Callback for /joy topic
 *
 * Runs upon each receipt of msg from /joy. Activates, disactivates,
 and estops based on button inputs. Changes state based on axis inputs.
 Passes through Twist messages based on joystick inputs unless estopped
 or disactivated, or message is same as message stored in drive_msg attr.
 *
 * @param[in] joy Message read from /joy topic
 */
void Teleop::joyCB(const sensor_msgs::Joy::ConstPtr &joy){
  //check for estop
  if(joy->buttons[estopButton] && !estop && !estopButtonFlag){
    activate(false);
    softestop(true);
    estopButtonFlag = true;
    return;
  }
  //check for estop button release
  if(!joy->buttons[estopButton] && !estop && estopButtonFlag){
    isActivated = false;
    estop = true;
    estopButtonFlag = false;
  }
  //check for un-estop
  if(joy->buttons[estopButton] && estop && !estopButtonFlag){
    softestop(false);
    estopButtonFlag = true;
  }
  //check for un-estop button release
  if(!joy->buttons[estopButton] && estop && estopButtonFlag){
    estop = false;
    estopButtonFlag = false;
  }

  //check if currently estopped
  if(!stop_msg.data){
    //check for statechange
    if(joy->axes[behaviorAxis] && !behaviorAxisFlag){
      incrementState(joy->axes[behaviorAxis]);
      behaviorAxisFlag = true;
    }
    //check for statechange axis release
    if(joy->axes[behaviorAxis] && behaviorAxisFlag){
      incrementState(joy->axes[behaviorAxis]);
      behaviorAxisFlag = false;
    }

    //check for activated
    if(joy->buttons[activateButton] && !isActivated && !activateButtonFlag){
      activate(true);
      activateButtonFlag = true;
    }
    //check for activated button release
    if(!joy->buttons[activateButton] && !isActivated && activateButtonFlag){
      isActivated = true;
      activateButtonFlag = false;
    }
    //check for disactivated
    if(joy->buttons[activateButton] && isActivated && !activateButtonFlag){
      activate(false);
      activateButtonFlag = true;
    }
    //check for disactivated button release
    if(!joy->buttons[activateButton] && isActivated && activateButtonFlag){
      isActivated = false;
      activateButtonFlag = false;
    }

    //check if tractor is activated
    if (isActivated){

      // generate and send twist message if unique
      if (drive_msg.twist.angular.z != joy->axes[0]
       || drive_msg.twist.linear.x != joy->axes[1]) {
        drive_msg.twist.angular.z = joy->axes[0];
        drive_msg.twist.linear.x =  joy->axes[1];
        drivemsg_pub.publish(drive_msg);
      }
    }
  }
}

/*
 * @brief publishes software estop command
 * @param[in] stop State of the softestop to publish
 */
void Teleop::softestop(bool stop){
  stop_msg.data = stop;
  softestop_pub.publish(stop_msg);
}

/*
 * @brief publishes activation command
 * @param[in] act State of the activate function to publish
 */
void Teleop::activate(bool act){
  activate_msg.data = act;
  activate_pub.publish(activate_msg);
}

/*
 * @brief publishes behavior statechange command
 * @param[in] behavior state to publish
 */
 void Teleop::state(int state){
   state_msg.data = state;
   state_pub.publish(state_msg);
 }

/*
 * @brief increments state in given direction
 *
 * Updates state to next numeric state or previous numeric state.
 * Implements wrapping of states, so incrementing down from 0th
 * state updates to nth state.
 * TODO(connor@students): Remove hardcoded number of states
 *
 * @param[in] direction to increment based on sign
 */
void Teleop::incrementState(float dir) {
  int s = state_msg.data;
  if(dir < 0) {
    s = state_msg.data + 1;
    if (s > 2) s = 0;
  }
  if(dir > 0) {
    s = state_msg.data - 1;
    if (s < 0) s = 2;
  }
  state(s);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "teleop");
  Teleop t;
  ros::spin();
}
