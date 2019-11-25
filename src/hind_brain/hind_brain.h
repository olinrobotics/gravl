/**
  hind_brain.h
  Purpose: Hindbrain libraries, constants, tuning parameters

  @author Connor Novak
  @email connor.novak@students.olin.edu
  @version 0.0 18/10/24
*/

#ifndef HIND_BRAIN_H
#define HIND_BRAIN_H

// Libraries
#include "RoboClaw.h"                       // Used for motor controller interface
#include <Arduino.h>                        // Used for Arduino functions
#include "ros.h"                            // Used for rosserial communication
#include "ackermann_msgs/AckermannDrive.h"  // Used for rosserial steering message
#include "geometry_msgs/Pose.h"             // Used for rosserial hitch message
#include "std_msgs/Empty.h"                 // Used for watchdog hf connection monitor
#include "estop.h"                          // Used to implement estop class
#include "soft_switch.h"                    // Used to implement auto switch
#include <Encoder.h>                        // Used for hitch height feedback
#include <std_msgs/Float64.h>

// Arduino Pins
const byte AUTO_LED_PIN = 3;
const byte ESTOP_PIN = 2;
const byte HITCH_ENC_A_PIN = 18;
const byte HITCH_ENC_B_PIN = 19;


// Roboclaw Constants
#define RC1_ADDRESS 0x80  // In front box, for steering and driving // NOTE: needs to change for multiple
#define RC2_ADDRESS 0x80  // In back of tractor, for hitch          // roboclaws on one serial port
// Timeout below equivalent to 10 ms
#define RC_TIMEOUT 10000

// General Constants
#define DEBUG True
#define WATCHDOG_TIMEOUT 250

// Velocity Motor Ranges
const int VEL_CMD_MIN = 1400;       // Roboclaw cmd for min speed
const int VEL_CMD_MAX = 0;          // Roboclaw cmd for max speed
const int VEL_MSG_MIN = 0;          // Ackermann msg min speed
const int VEL_MSG_MAX = 2;          // Ackermann msg max speed

// Steering Motor Ranges
const int STEER_CMD_LEFT = 500;        // Roboclaw cmd for max left turn
const int STEER_CMD_CENTER = 975;      // Roboclaw cmd for straight
const int STEER_CMD_RIGHT = 1275;      // Roboclaw cmd for max right turn
const int STEER_MSG_LEFT = 45;         // Ackermann msg min steering angle
const int STEER_MSG_CENTER = 0;        // Ackermann msg center steering angle
const int STEER_MSG_RIGHT = -30;       // Ackermann msg max steering angle

// Hitch Actuator Ranges
const int H_ACTUATOR_MAX = 1593; // Retracted Actuator - Move hitch up
const int H_ACTUATOR_MIN = 656;  // Extended Actuator - Move hitch down
const int H_ACTUATOR_CENTER = 1215
const int H_ACTUATOR_RANGE = H_ACTUATOR_MAX-H_ACTUATOR_MIN;
// Encoder
const float ENC_STOP_THRESHOLD = 0.0381; // Threshold of blade accuracy to stop in meters

// function prototypes
void ackermannCB(const ackermann_msgs::AckermannDrive&);
void hitchCB(const geometry_msgs::Pose&);
void watchdogCB(const std_msgs::Empty&);
void stopEngine();
void eStop();
void eStart();
void checkSerial(ros::NodeHandle *nh);
void updateCurrDrive();
void updateCurrHitchPose();
void updateRoboClaw(int velMsg, int steerMsg, int hitchMsg);
void stopRoboClaw(RoboClaw *rc1, RoboClaw *rc2);
int steerAckToCmd(float ack_steer);
int velAckToCmd(float ack_vel);
float mapPrecise(float x, float inMin, float inMax, float outMin, float outMax);
int computeHitchMsg();

#endif
