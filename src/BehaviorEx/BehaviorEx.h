/*
 * @file BehaviorEx.cpp
 * @brief Example behavior, publishes command received on /ex_topic
 *
 * Subscribes to /ex_topic, publishes received msgs to state controller via
 * /cmd_behavior
 * @author Connor Novak
 * @date 2018-11-13
 */

#ifndef BEHAVIOR_EX_H
#define BEHAVIOR_EX_H

#include <ros/ros.h>
#include <gravl/TwistLabeled.h>

class BehaviorEx {
  public:
    explicit BehaviorEx();
  private:
    ros::NodeHandle n;
    ros::Subscriber twist_subscriber;
    ros::Publisher twist_publisher;

    gravl::TwistLabeled current_message;

    void twistCB(const geometry_msgs::Twist& msg);
};

#endif
