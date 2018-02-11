/******************************************************************************
 * DriveState
 * @file DriveState.cpp
 * @author Carl Moser
 * @email carl.moser@students.olin.edu
 * @version     1.0
 *
 * This takes in messages from teledrive and autodrive and publishes them to 
 * drive based on the value published to auto (default-false)
 * auto = true:  autodrive -> drive
 * auto = false: teledrive -> drive
 *
 ******************************************************************************/


#include "DriveState.h"

/*
 * DriveState constructor
 */
DriveState::DriveState(){
  state =  n.subscribe("auto", 10, &DriveState::stateCB, this);
  telesub = n.subscribe("teledrive", 10, &DriveState::teleCB, this);
  drivepub = n.advertise<ackermann_msgs::AckermannDrive>("drive", 1000);
}

/*
 * Callback function for auto subscriber
 */
void DriveState::stateCB(const std_msgs::Bool &msg){
  if(msg.data){
    telesub.shutdown();
    autosub = n.subscribe("autodrive", 10, &DriveState::autoCB, this);
  }
  else{
    autosub.shutdown();
    telesub = n.subscribe("teledrive", 10, &DriveState::teleCB, this);
  }
}

/* 
 * Callback function for teledrive subscriber - publishes to drivepub
 */
void DriveState::teleCB(const ackermann_msgs::AckermannDrive &msg){
  drivepub.publish(msg);
}

/* 
 * Callback function for autodrive subscriber - publishes to drivepub
 */
void DriveState::autoCB(const ackermann_msgs::AckermannDrive &msg){
  drivepub.publish(msg);
}

// main function
int main(int argc, char **argv)
{
	ros::init(argc, argv, "state");
  DriveState ds;
  ros::spin();
}
