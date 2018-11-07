#pragma once

#include <gravl/DiffDrive.h>
#include <tf2_ros/transform_broadcaster.h>

class EncOdom
{
public:
  EncOdom();

private:
  void callback(const gravl::DiffDrive::ConstPtr& msg);

  ros::NodeHandle nh;
  tf2_ros::TransformBroadcaster br;
  ros::Subscriber sub;
  geometry_msgs::TransformStamped transformStamped;
  ros::Time then;
};
