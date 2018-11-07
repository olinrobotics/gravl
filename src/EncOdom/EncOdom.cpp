#include "EncOdom.h"

EncOdom::EncOdom()
  : sub(nh.subscribe("diffdrive_raw", 100, &EncOdom::callback, this))
  , then(ros::Time::now())
{}

void EncOdom::callback(const gravl::DiffDrive::ConstPtr& msg)
{
  const auto now = ros::Time::now();
  const auto omega = (msg->rvel - msg->lvel) / msg->wheelbase;
  const auto xdot = (msg->rvel + msg->lvel) / 2;
}

int main(int argc, char** argv)
{
}
