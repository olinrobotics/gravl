#include "EncOdom.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

EncOdom::EncOdom()
  : sub(nh.subscribe("diffdrive_raw", 100, &EncOdom::callback, this))
  , then(ros::Time::now())
{
  transformStamped.header.frame_id = "odom";
  transformStamped.child_frame_id = "base_link";
}

void EncOdom::callback(const gravl::DiffDrive::ConstPtr& msg)
{
  const auto now = ros::Time::now();
  const auto angularVel = (msg->rvel - msg->lvel) / msg->wheelbase;
  const auto tangentialVel = (msg->rvel + msg->lvel) / 2;
  const auto dt = (now - then).toSec();

  tf2::Quaternion oldOrientation;
  tf2::convert(transformStamped.transform.rotation, oldOrientation);
  const auto newOrientation = oldOrientation * tf2::Vector3(0, 0, angularVel * dt);
  tf2::convert(newOrientation, transformStamped.transform.rotation);

  const auto delta_position = tf2::quatRotate(newOrientation, tf2::Vector3(0, 0, tangentialVel * dt));
  tf2::convert(delta_position, transformStamped.transform.translation);

  br.sendTransform(transformStamped);
  then = now;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "enc_odom");
  EncOdom encOdom;
  ros::spin();
}
