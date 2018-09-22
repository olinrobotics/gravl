#pragma once
#define IMU_SAFETY_COHSTANTS_H 1

#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include "gravl/ImuSafety.h"

const double max_alpha_z = 10;

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

      ros::NodeHandle n;
      ros::Publisher pub;
      ros::Subscriber sub;
      double omega_z0;
      ros::Time t0;
      gravl::ImuSafety pub_val;
      ros::Rate rate;
    };
  }
}
