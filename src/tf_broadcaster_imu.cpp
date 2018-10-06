#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

#define _USE_MATH_DEFINES

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster tb;

  while(n.ok())
    {
      tb.sendTransform
        (
         tf::StampedTransform
         (
          tf::Transform
          (
           tf::createQuaternionFromRPY(M_PI / 2, 0 ,M_PI / 2),
           tf::Vector3(-.6, -.2, 1.4)
           ),
          ros::Time::now(), "base_link", "base_imu"
          )
         );

      r.sleep();
    }
}
