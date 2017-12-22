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
Teleop::Teleop(){
  joystick = n.subscribe("/joy", 10, &Teleop::joyCB, this);
  teledrive = n.advertise<ackermann_msgs::AckermannDrive>("teledrive", 1000);
  softestop = n.advertise<std_msgs::Bool>("softestop", 1000);
  autonomous = n.advertise<std_msgs::Bool>("auto", 1000);
}

/*
 * The callback for the gamepad input
 */
void Teleop::joyCB(const sensor_msgs::Joy::ConstPtr &joy){
  //check for estop
  if(joy->buttons[1]){
    auto_pub(false);
    stop_pub(true);
    return;
  }
  //check for un-estop
  if(joy->buttons[0]){
    stop_pub(false);
  }

  //check if currently estopped
  if(!stop_msg.data){
    if(joy->buttons[5]){
      auto_pub(true);
    }
    else if(joy->buttons[4]){
      auto_pub(false);
    }
    else{
      drive_msg.steering_angle = 45*joy->axes[3];
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
