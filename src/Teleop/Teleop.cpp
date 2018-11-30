/*
 * Joystick Command Node
 * @file Teleop.cpp
 * @author Carl Moser
 * @email carl.moser@students.olin.edu
 * @maintainer Kubo
 * @email olingravl@gmail.com
 * @version 1.1.0
 *
 * Takes input from controller specified by controllerType param, outputs to
 * main state controller - handles movement messages and state change commands
 */

#include "Teleop.h"

/*
 * Constructor - advertises and subscribes topics
 */
Teleop::Teleop()
: estop(false)
, joystick_sub(n.subscribe("/joy", 10, &Teleop::joyCB, this))
, softestop_pub(n.advertise<std_msgs::Bool>("/softestop", 1))
, activate_pub(n.advertise<std_msgs::Bool>("/scin_activate", 1))
, drivemsg_pub(n.advertise<gravl::TwistLabeled>("/scin_behavior", 1))
, state_pub(n.advertise<std_msgs::UInt8>("/scin_state", 1))
, isActivated(false)
, estopButtonFlag(false)
, activateButtonFlag(false)
, n("~")
, activateButton(0)
, estopButton(1)
{
  drive_msg.label = 0; // TODO: Actually get label
  n.param<std::string>("controllerType", controllerType, "gamepad");
  if (controllerType == "gamepad"){
    activateButton = 0;
    estopButton = 1;
  }
  if (controllerType == "joystick"){
    activateButton = 6;
    estopButton = 0;
  }
}

/*
 * The callback for the gamepad input
 */
void Teleop::joyCB(const sensor_msgs::Joy::ConstPtr &joy){
  //check for estop
  if(joy->buttons[estopButton] && !estop && !estopButtonFlag){
    activate(false);
    softestop(true);
    estopButtonFlag = true;
    return;
  }
  //check for button release
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
  //check for button release
  if(!joy->buttons[estopButton] && estop && estopButtonFlag){
    estop = false;
    estopButtonFlag = false;
  }

  //check if currently estopped
  if(!stop_msg.data){
    //check for activated
    if(joy->buttons[activateButton] && !isActivated && !activateButtonFlag){
      activate(true);
      activateButtonFlag = true;
    }
    //check for button release
    if(!joy->buttons[activateButton] && !isActivated && activateButtonFlag){
      isActivated = true;
      activateButtonFlag = false;
    }
    //check for unautonomous
    if(joy->buttons[activateButton] && isActivated && !activateButtonFlag){
      activate(false);
      activateButtonFlag = true;
    }
    //check for button release
    if(!joy->buttons[activateButton] && isActivated && activateButtonFlag){
      isActivated = false;
      activateButtonFlag = false;
    }
    //check if tractor is activated
    if (isActivated){
      drive_msg.twist.angular.z = joy->axes[0];
      drive_msg.twist.linear.x =  joy->axes[1];
      drivemsg_pub.publish(drive_msg);
    }
  }
}

/*
 * Publishes to the softestop topic
 * @param[in] stop State of the softestop to publish
 */
void Teleop::softestop(bool stop){
  stop_msg.data = stop;
  softestop_pub.publish(stop_msg);
}

/*
 * Publishes to the auto topic
 * @param[in] act State of the activate function to publish
 */
void Teleop::activate(bool act){
  activate_msg.data = act;
  activate_pub.publish(activate_msg);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "teleop");
  Teleop t;
  ros::spin();
}
