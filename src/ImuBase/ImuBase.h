/*
 * @file ImuBase.h
 * @author Kawin Nikomborirak
 * @date 2018-10-03
 *
 * Subscribes to /imu/data and publishes the transformed imu message
 * to /imu_base.
 */

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>

class ImuBase
{
public:
  ImuBase();

  /* Run the node. */
  void spin();

private:
  void callback(const sensor_msgs::Imu::ConstPtr& msg);

  sensor_msgs::Imu pub_val;
  ros::NodeHandle n;
  tf::TransformListener tl;
  tf::StampedTransform transform;
  const ros::Publisher pub;
  const ros::Subscriber sub;
  ros::Rate rate;
};
