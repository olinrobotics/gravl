#pragma once
#define IMU_SAFETY_COHSTANTS_H 1

#include "ros/ros.h"

const double max_alpha_z = 10;

namespace safety
{
  namespace imu
  {
    class imu_safety
    {
    public:
      imu_safety();
    private:
      ros::NodeHandle n;
      ros::Publisher pub;
      ros::Subscriber sub;
    };
  }
}
