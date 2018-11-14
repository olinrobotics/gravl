#include "MainState.h"

MainState()
  : pub(n.advertise<sensor_msgs::Imu>("imu_base", 1000))
  , sub(n.subscribe("/cmd_behavior", 10, &MainState::MainState::callback, this))
  , rate(ros::Rate(10))
  , tl(tfBuffer)
{
}

void MainState::behavior_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
  tf2::doTransform(msg->orientation, pub_val.orientation, transform);
  tf2::doTransform(msg->angular_velocity, pub_val.angular_velocity, transform);
  tf2::doTransform(msg->linear_acceleration, pub_val.linear_acceleration, transform);
}

void MainState::estop_cb()

void MainState::spin()
{
  while (ros::ok())
    {

      ros::spinOnce();
      rate.sleep();
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MainState");
  MainState mainstate;
  MainState.spin();
}
