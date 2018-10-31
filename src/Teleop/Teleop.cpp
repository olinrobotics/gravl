/******************************************************************************
 * Teleop
 * @file Teleop.cpp
 * @author Carl Moser
 * @email carl.moser@students.olin.edu
 * @version     1.0
 *
 * Takes the input from a Logitech f310 gamepad and publishes to various topics
 *
 ******************************************************************************/


#include "Teleop.h"

/*
 * Constructor - advertises and subscribes topics
 */
Teleop::Teleop()
: estop(false)
, isAutonomous(false)
, estopButtonFlag(false)
, autoButtonFlag(false)
, n("~")
, autoButton(0)
, estopButton(1)
{

  joystick = n.subscribe("/joy", 10, &Teleop::joyCB, this);
  teledrive = n.advertise<ackermann_msgs::AckermannDrive>("/teledrive", 1000);
  softestop = n.advertise<std_msgs::Bool>("/softestop", 1000);
  autonomous = n.advertise<std_msgs::Bool>("/auto", 1000);
  n.param<std::string>("controllerType", controllerType, "gamepad");
  if (controllerType == "gamepad"){
    autoButton = 0;
    estopButton = 1;
  }
  if (controllerType == "joystick"){
    autoButton = 6;
    estopButton = 0;
  }
}

/*
 * The callback for the gamepad input
 */
void Teleop::joyCB(const sensor_msgs::Joy::ConstPtr &joy){
  //check for estop
  if(joy->buttons[estopButton] && !estop && !estopButtonFlag){
    auto_pub(false);
    stop_pub(true);
    estopButtonFlag = true;
    return;
  }
  //check for button release
  if(!joy->buttons[estopButton] && !estop && estopButtonFlag){
    isAutonomous = false;
    estop = true;
    estopButtonFlag = false;
  }
  //check for un-estop
  if(joy->buttons[estopButton] && estop && !estopButtonFlag){
    stop_pub(false);
    estopButtonFlag = true;
  }
  //check for button release
  if(!joy->buttons[estopButton] && estop && estopButtonFlag){
    estop = false;
    estopButtonFlag = false;
  }

  //check if currently estopped
  if(!stop_msg.data){
    //check for autonomous
    if(joy->buttons[autoButton] && !isAutonomous && !autoButtonFlag){
      auto_pub(true);
      autoButtonFlag = true;
    }
    //check for button release
    if(!joy->buttons[autoButton] && !isAutonomous && autoButtonFlag){
      isAutonomous = true;
      autoButtonFlag = false;
    }
    //check for unautonomous
    if(joy->buttons[autoButton] && isAutonomous && !autoButtonFlag){
      auto_pub(false);
      autoButtonFlag = true;
    }
    //check for button release
    if(!joy->buttons[autoButton] && isAutonomous && autoButtonFlag){
      isAutonomous = false;
      autoButtonFlag = false;
    }
    //check if autonomous is running
    if (!isAutonomous){
      drive_msg.steering_angle = 45*joy->axes[0];
      drive_msg.speed = 2*joy->axes[1];
      teledrive.publish(drive_msg);
    }
  }
}

/*
 * Publishes to the softestop topic
 * @param[in] stop State of the softestop to publish
 */
void Teleop::stop_pub(bool stop){
  stop_msg.data = stop;
  softestop.publish(stop_msg);
}

/*
 * Publishes to the auto topic
 * @param[in] aut State of the autonomous functions to publishs
 */
void Teleop::auto_pub(bool aut){
  autonomous_msg.data = aut;
  autonomous.publish(autonomous_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "teleop");
  Teleop t;
  ros::spin();
}
