#include "ImuSafety.h"
#include <tf/tf.h>

ImuSafety::ImuSafety()
  : pub(n.advertise<gravl::ImuSafety>("imu_safe", 1000))
  , sub(n.subscribe("/imu/data", 1000, &ImuSafety::ImuSafety::callback, this))
  , rate(ros::Rate(10))
{
  // Default to 10 degrees
  n.param<double>("maxRoll", max_roll, .1745);
}

void ImuSafety::callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  tf::Quaternion q_orig;
  quaternionMsgToTF(msg->orientation, q_orig);
  double dummy_var;
  pub_val.danger = false;
  ((tf::Matrix3x3) q_orig).getRPY(pub_val.theta, dummy_var, dummy_var);
  pub_val.danger = abs(pub_val.theta) > max_roll;
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
  imu_safety.spin();
}
