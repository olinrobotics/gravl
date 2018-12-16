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
 * @version 1.2.0
 *
 * Takes input from controller specified by controllerType param, outputs to
 * main state controller - handles movement messages and state change commands
 *
 * @author Carl Moser
 * @email carl.moser@students.olin.edu
 */

#include "Teleop.h"
#include <std_msgs/String.h>

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
, state_pub(n.advertise<std_msgs::String>("/state_controller/cmd_state", 1))
, estop(false)
, isActivated(false)
, activateButton(0)
, estopButton(1)
, behaviorAxis(7)
, estopButtonFlag(false)
, activateButtonFlag(false)
, behaviorAxisFlag(false)
{
  // Init and sort behavior list, current behavior
  behaviors = getBehaviors(n);
  sort(behaviors.begin(), behaviors.end(), compareBehaviorId);
  state_behavior = behaviors[0];

  drive_msg.label = "teleop";
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

void Teleop::joyCB(const sensor_msgs::Joy::ConstPtr &joy){
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

void Teleop::softestop(bool stop){
  /*
   * @brief publishes software estop command
   * @param[in] stop State of the softestop to publish
   */
  stop_msg.data = stop;
  softestop_pub.publish(stop_msg);
}

void Teleop::activate(bool act){
  /*
   * @brief publishes activation command
   * @param[in] act State of the activate function to publish
   */
  activate_msg.data = act;
  activate_pub.publish(activate_msg);
}

void Teleop::state(Behavior behavior){
   /*
    * @brief publishes behavior statechange command
    * @param[in] behavior state to publish
    */
   state_behavior = behavior;
   auto msg = std_msgs::String();
   msg.data = behavior.name.c_str();
   state_pub.publish(msg);
 }

int Teleop::incrementState(float dir) {
  /*
   * @brief increments state in given direction
   *
   * Updates state to next numeric state or previous numeric state.
   * Implements wrapping of states, so incrementing down from 0th
   * state updates to nth state.
   * TODO(connor@students): Remove hardcoded number of states
   *
   * @param[in] direction to increment based on sign
   * return integer error code
   */
  int s = state_behavior.id;
  Behavior b_new;

  // If incrementing
  if(dir > 0) b_new = behaviors[(s + 1) % behaviors.size()];

  // If decrementing (wrap to remove vals < 0)
  else if (dir < 0) {
    if (s==0) s = behaviors.size();
    b_new = behaviors[(s - 1)];
  } else return 1;

  state(b_new);
  return 0;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "teleop");
  Teleop t;
  ros::spin();
}
