#include "ImuSafety.h"

using namespace safety::imu;

const double max_alpha_z = 2;

ImuSafety::ImuSafety()
  : pub(n.advertise<gravl::ImuSafety>("safe_alpha", 1000))
  , sub(n.subscribe("/imu/data_raw", 1000, &ImuSafety::ImuSafety::callback, this))
  , t0(ros::Time::now())
  , omega_z0(0)
  , rate(ros::Rate(10))
{
  n.param<double>("maxAlphaZ", max_alpha_z, 10);
}

void ImuSafety::callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  const auto t = msg->header.stamp;
  const auto omega_z = msg->angular_velocity.z;

  const auto dt = (t - t0).toSec();
  const auto d_omega_z = omega_z - omega_z0;
  const auto alpha_z = d_omega_z / dt;

  pub_val.alpha = alpha_z;
  pub_val.danger = alpha_z < max_alpha_z;

  t0 = t;
  omega_z0 = omega_z;
}

void ImuSafety::spin()
{
  while (ros::ok())
    {
      pub.publish(pub_val);
      ros::spinOnce();
      rate.sleep();
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ImuSafety");
  ImuSafety imu_safety;
}
