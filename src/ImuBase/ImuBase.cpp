#include "ImuBase.h"
#include <tf/tf.h>

ImuBase::ImuBase()
  : pub(n.advertise<sensor_msgs::Imu>("imu_safe", 1000))
  , sub(n.subscribe("/imu/data", 1000, &ImuBase::ImuBase::callback, this))
  , rate(ros::Rate(10))
{}

void ImuBase::callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  tf::Quaternion q_orig;
  quaternionMsgToTF(msg->orientation, q_orig);
  const auto orientation = transform.getRotation() * q_orig;
  double dummy_var;
  pub_val.danger = false;
  ((tf::Matrix3x3) orientation).getRPY(pub_val.theta, dummy_var, dummy_var);
  pub_val.danger = abs(pub_val.theta) > max_roll;
}

void ImuBase::spin()
{
  while (ros::ok())
    {
      try
        {
          tl.lookupTransform("/base_link", "/base_imu", ros::Time(0), transform);
        }
      catch (tf::TransformException ex)
        {
          ROS_ERROR("%s", ex.what());
          ros::Duration(1).sleep();
          continue;
        }

      pub.publish(pub_val);
      ros::spinOnce();
      rate.sleep();
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ImuBase");
  ImuBase imu_safety;
  imu_safety.spin();
}
