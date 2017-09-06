#include "State.h"

State::State(){
  state =  n.subscribe("auto", 10, &State::stateCB, this);
  telesub = n.subscribe("teledrive", 10, &State::teleCB, this);
  drivepub = n.advertise<ackermann_msgs::AckermannDrive>("drive", 1000);
}

void State::stateCB(const std_msgs::Bool &msg){
  if(msg.data){
    telesub.shutdown();
    waysub = n.subscribe("waydrive", 10, &State::wayCB, this);
  }
  else{
    waysub.shutdown();
    telesub = n.subscribe("teledrive", 10, &State::teleCB, this);
  }
}

void State::teleCB(const ackermann_msgs::AckermannDrive &msg){
  drivepub.publish(msg);
}

void State::wayCB(const ackermann_msgs::AckermannDrive &msg){
  drivepub.publish(msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "state");
  State s;
  ros::spin();
}
