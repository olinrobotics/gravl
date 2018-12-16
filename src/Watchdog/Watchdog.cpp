#include "Watchdog.h"

Watchdog::Watchdog()
: rate(ros::Rate(10))
, clockpub(n.advertise<std_msgs::Empty>("safety_clock", 1))
{
}

void Watchdog::spin() {
  //Publishes empty message while roscore is running

  while(ros::ok()) {
    clockpub.publish(tick);
    ros::spinOnce();
    rate.sleep();
  }

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "watchdog");
  Watchdog chip;
  chip.spin();
}
