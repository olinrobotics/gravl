#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <gravl/ImuSafety.h>

/*
 * Subscribes to /imu/data and publishes a roll of the tractor and
 * whether or not the roll is dangerous.
 */
class ImuSafety
{
public:
  ImuSafety();

  /* Run the node. */
  void spin();

private:
  void callback(const sensor_msgs::Imu::ConstPtr& msg);

  gravl::ImuSafety pub_val;
  ros::NodeHandle n;
  const ros::Publisher pub;
  const ros::Subscriber sub;
  ros::Rate rate;

  /* The maximum tractor roll that is considered 'safe'. */
  double max_roll;
};
