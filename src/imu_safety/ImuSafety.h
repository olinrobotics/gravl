#pragma once

#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include "gravl/ImuSafety.h"

namespace safety
{
  namespace imu
  {
    class ImuSafety
    {
    public:
      ImuSafety();
    private:
      void callback(const sensor_msgs::Imu::ConstPtr& msg);
      void spin();

      double omega_z0;
      ros::Time t0;
      gravl::ImuSafety pub_val;
      ros::NodeHandle n;
      ros::Publisher pub;
      ros::Subscriber sub;
      ros::Rate rate;
      double max_alpha_z;
    };
  }
}
