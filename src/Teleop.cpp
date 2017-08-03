#include "Teleop.h"

Teleop::Teleop(){
  joystick = n.subscribe("joy", 10, &Teleop::joyCB, this);
  drive = n.advertise<ackermann_msgs::AckermannDrive>("teledrive", 1000);
  softestop = n.advertise<std_msgs::Bool>("softestop", 1000);
  autonomous = n.advertise<std_msgs::Bool>("auto", 1000);
}

void Teleop::joyCB(const sensor_msgs::Joy::ConstPtr &joy){
  if(joy->buttons[1]){
    auto_pub(false);
    stop_pub(true);
    return;
  }
  if(joy->buttons[0]){
    stop_pub(false);
  }

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
      drive.publish(drive_msg);
    }
  }
}

void Teleop::stop_pub(bool stop){
  stop_msg.data = stop;
  softestop.publish(stop_msg);
}

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
