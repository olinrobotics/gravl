/******************************************************************************
 * Hemisphere 
 * @file State.cpp
 * @author Carl Moser
 * @email carl.moser@students.olin.edu
 * @version     1.0
 *
 * This takes the true heading from the Hemisphere GPS and publishes it
 *
 ******************************************************************************/


#include "State.h"

// State class
State::State(){
  state =  n.subscribe("auto", 10, &State::stateCB, this);
  telesub = n.subscribe("teledrive", 10, &State::teleCB, this);
  drivepub = n.advertise<ackermann_msgs::AckermannDrive>("drive", 1000);
}

/*
 * Callback function for auto subscriber
 * DESC: Shuts down one topic and opens another based on state of system (auto/teledrive)
 */
void State::stateCB(const std_msgs::Bool &msg){
  if(msg.data){
    telesub.shutdown();
    autosub = n.subscribe("autodrive", 10, &State::autoCB, this);
  }
  else{
    autosub.shutdown();
    telesub = n.subscribe("teledrive", 10, &State::teleCB, this);
  }
}

/* Callback function for teledrive subscriber
 * DESC: publishes data to
 *
 */
void State::teleCB(const ackermann_msgs::AckermannDrive &msg){
  drivepub.publish(msg);
}

void State::autoCB(const ackermann_msgs::AckermannDrive &msg){
  drivepub.publish(msg);
}

// main function
int main(int argc, char **argv)
{
	ros::init(argc, argv, "state");
  State s;
  ros::spin();
}
