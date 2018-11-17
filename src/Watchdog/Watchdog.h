/*
 * @file Watchdog.h
 * @author Connor Novak
 * @date 2018-11-14
 *
 * Publishes high-frequency topic for watchdog on hindbrain to monitor.
 */

#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <ros/ros.h>
#include<std_msgs/Empty.h>

class Watchdog {

public:
  explicit Watchdog();
  void spin();

private:
  ros::NodeHandle n;
  ros::Publisher clockpub;
  std_msgs::Empty tick;
  ros::Rate rate;

};
#endif
