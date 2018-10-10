#include "imu_base.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_base");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<sensor_msgs::Imu>("imu_base", 10);
  tf::TransformListener tl;
  ros::Rate rate(10);
  while (n.ok())
    {
      tf::StampedTransform transform;

      try tl.lookupTransform("/")
    }
}
