/*
 * @file BehaviorEx.cpp
 * @author Connor Novak
 * @date 2018-12-12
 *
 * Subscribes to /cmd_behavior, publishes msgs to /cmd_vel based on tractor's
 * current state.
 */

 #include "BehaviorEx.h"          // header
 #include <ros/console.h>         // used for logging
 #include <geometry_msgs/Twist.h> // node lisens to twist messages
 #include <gravl/TwistLabeled.h>  // node publishes twistlabeled messages
 #include <std_msgs/String.h>

 BehaviorEx::BehaviorEx() {
    twist_subscriber = n.subscribe("/ex_topic", 1, &BehaviorEx::BehaviorEx::twistCB, this);
    twist_publisher = n.advertise<gravl::TwistLabeled>("/state_controller/cmd_behavior", 1);
    current_message.label = "example";
 }

void BehaviorEx::twistCB(const geometry_msgs::Twist& msg) {
  /*
   * @brief callback function called for every command message received
   *
   * @param[in] twist message from subscriber
   * param[out] updates current_message class attribute
   */

   current_message.twist = msg;
   twist_publisher.publish(current_message);

}

int main (int argc, char** argv) {
  ros::init(argc, argv, "BehaviorEx");  //Initialize the ROS node
  BehaviorEx behavior;                  // Initialize a behavior class instance
  ros::spin();                          // Continuously update the ROS loop
}
