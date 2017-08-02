#include "Teleop.h"

Teleop::Teleop(){
  joystick = n.subscribe("joy", 10, &Teleop::joyCB, this);
  drive = n.advertise<ackermann_msgs::AckermannDrive>("drive", 1000);
  softestop = n.advertise<std_msgs::Bool>("softestop", 1000);
}

void Teleop::joyCB(const sensor_msgs::Joy::ConstPtr &joy){
  if(joy->buttons[1]){
    stop_msg.data = true;
    softestop.publish(stop_msg);
    return;
  }
  else if(joy->buttons[0]){
    stop_msg.data = false;
    softestop.publish(stop_msg);
  }
  else{
    if(!stop_msg.data){
      drive_msg.steering_angle = 45*joy->axes[3];
      drive_msg.speed = 2*joy->axes[1];
      drive.publish(drive_msg);
    }
  }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "teleop");
  Teleop t;
  ros::spin();
}
