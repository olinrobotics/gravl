#include "ImuBase.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

ImuBase::ImuBase()
  : pub(n.advertise<sensor_msgs::Imu>("imu_base", 1000))
  , sub(n.subscribe("/imu/data", 1000, &ImuBase::ImuBase::callback, this))
  , rate(ros::Rate(10))
  , tl(tfBuffer)
{
}

void ImuBase::callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  tf2::doTransform(msg->orientation, pub_val.orientation, transform);
  tf2::doTransform(msg->angular_velocity, pub_val.angular_velocity, transform);
  tf2::doTransform(msg->linear_acceleration, pub_val.linear_acceleration, transform);
}

void ImuBase::spin()
{
  while (ros::ok())
    {
      try
        {
          transform = tfBuffer.lookupTransform("base_link", "base_imu", ros::Time(0));
        }
      catch (tf2::TransformException &ex)
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
  ImuBase imu_base;
  imu_base.spin();
}
