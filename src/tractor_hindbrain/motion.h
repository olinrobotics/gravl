#ifndef MOTION_H
#define MOTION_H

#include "RoboClaw.h"
#include "ros.h"
#include "ackermann_msgs/AckermannDrive.h"
#include <gravl/RoboClawStats.h>

#define MOTION_ROBOCLAW_SERIAL Serial1
#define MOTION_ROBOCLAW_ADDRESS 0x80
#define MOTION_ROBOCLAW_BAUD 38400

class Motion{
public:
  explicit Motion(ros::NodeHandle *nh);
  void homeActuators();
  void updateRoboClaw();

private:
  // Constants
  const int VEL_HIGH = 2048;
  const int VEL_LOW = 190;
  const int VEL_CONTROL_RANGE = 2;
  const int VEL_HOME = VEL_HIGH;                     // Neutral position; vel=0
  const int STEER_HIGH = 1200;
  const int STEER_LOW = 600;
  const int STEER_CONTROL_RANGE = 90;
  const int STEER_HOME = (STEER_HIGH + STEER_LOW)/2; // Neutral position; steer angle=0
  const int VEL_STEP = (VEL_HIGH - VEL_LOW)/10;      // (HIGH-LOW/FIDELITY)
  const int STEER_STEP = (STEER_HIGH - STEER_LOW)/1; // (HIGH-LOW/FIDELITY)

  // Current messages
  float vel_msg;
  float steer_msg;

  // Motor Positions
  uint32_t cur_vel_pos;
  uint32_t cur_steer_pos;
  uint32_t new_vel_pos;
  uint32_t new_steer_pos;

  // Functions
  void stepActuator(uint32_t *new_pos, uint32_t cur_pos, int step);
  void velConvert();
  void steerConvert();

  // RoboClaw and ROS
  RoboClaw *roboclaw;
  ros::Subscriber<ackermann_msgs::AckermannDrive, Motion> *cmd;
  void ackermannCallback(const ackermann_msgs::AckermannDrive &drive);
};

#endif //MOTION_H
