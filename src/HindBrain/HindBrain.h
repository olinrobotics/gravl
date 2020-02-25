/*
  @file HindBrain.h
  Purpose: Config file for HindBrain.ino
*/

#ifndef HIND_BRAIN_H
#define HIND_BRAIN_H

// https://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World
#define USE_USBCON

// Libraries

#include <ros.h>                            // rosserial library
#include <std_msgs/Empty.h>                 // Watchdog hf connection monitor
#include <std_msgs/String.h>                // rosserial String msg
#include <geometry_msgs/Pose.h>             // rosserial Hitch msg

#include "Estop.h"                          // OAK Estop
#include "SoftSwitch.h"                     // OAK SoftSwitch
#include "RoboClaw.h"                       // Motor controller API
#include "ackermann_msgs/AckermannDrive.h"  // rosserial Steering msg

// Arduino Pins
const byte SERIAL_PIN_0 = 0;      // Using either pin for input causes errors
const byte SERIAL_PIN_1 = 1;
const byte ESTOP_RELAY_PIN = 2;
const byte AUTO_LIGHT_PIN = 3;
const byte ESTOP_SENSE_PIN = 4;   // Not implemented
const byte HITCH_ENC_A_PIN = 18;
const byte HITCH_ENC_B_PIN = 19;

// General Constants
const bool DEBUG = false;
const int WATCHDOG_TIMEOUT = 250; // ms
#define SERIAL_BAUD_RATE 115200   // hz

// Roboclaw Constants
// Note: addresses must only be unique for 2+ roboclaws on 1 serial port
#define RC1_ADDRESS 0x80    // Vehicle Control Unit
#define RC2_ADDRESS 0x80    // Implement Control Unit
#define RC_BAUD_RATE 38400  // hz

const unsigned int RC_TIMEOUT = 10000;  // us
const byte ESTOP_DEBOUNCE_TIME = 50; // ms

// Velocity Motor Ranges
const int VEL_CMD_REV = 1050;   // Roboclaw cmd for max reverse speed
const int VEL_CMD_STOP = 1115;  // . . .            stopped
const int VEL_CMD_FWD = 1220;   // . . .            max forward speed
const int VEL_MSG_REV = -2;     // Ackermann msg for max reverse speed
const int VEL_MSG_STOP = 0;     // . . .             stopped
const int VEL_MSG_FWD = 2;      // . . .             max forward speed

// Initialize Roboclaw VEL_CMD thresholds w/ preprocessor macros
#if VEL_CMD_REV>VEL_CMD_FWD
  #define VEL_CMD_MAX VEL_CMD_REV
  #define VEL_CMD_MIN VEL_CMD_FWD
#else
  #define VEL_CMD_MAX VEL_CMD_FWD
  #define VEL_CMD_MIN VEL_CMD_REV
#endif

// Steering Motor Ranges
const int STEER_CMD_LEFT = 650;     // Roboclaw cmd for max left turn
const int STEER_CMD_CENTER = 1160;  // . . .            straight
const int STEER_CMD_RIGHT = 1600;   // . . .            max right turn
const int STEER_MSG_LEFT = 45;      // Ackermann msg leftmost steering angle
const int STEER_MSG_CENTER = 0;     // . . .         center . . .
const int STEER_MSG_RIGHT = -45;    // . . .         rightmost . . .

// Initialize Roboclaw STEER_CMD thresholds w/ preprocessor macros
#if STEER_CMD_LEFT>STEER_CMD_RIGHT
  #define STEER_CMD_MAX STEER_CMD_LEFT
  #define STEER_CMD_MIN STEER_CMD_RIGHT
#else
  #define STEER_CMD_MAX STEER_CMD_RIGHT
  #define STEER_CMD_MIN STEER_CMD_LEFT
#endif

// Hitch Actuator Ranges
const int H_ACTUATOR_MAX = 1660;    // Retracted Actuator - Move hitch up
const int H_ACTUATOR_MIN = 656;     // Extended Actuator - Move hitch down
const int H_ACTUATOR_CENTER = 1162; // Neutral Actuator - Stop moving hitch
const int H_ACTUATOR_RANGE = H_ACTUATOR_MAX - H_ACTUATOR_MIN;

// Encoder Constants
const float ENC_STOP_THRESHOLD = 0.0381;  // Blade accuracy stop threshold (m)

// Callback prototypes
void ackermannCB(const ackermann_msgs::AckermannDrive&);
void hitchCB(const geometry_msgs::Pose&);
void userInputCB(const std_msgs::String&);
void watchdogCB(const std_msgs::Empty&);

// Function prototypes
void checkSerial(ros::NodeHandle *nh);
void eStartTractor();
void eStopTractor();
void runStartupSequence();
void stopEngine();
void stopRoboClaw(RoboClaw *rc1, RoboClaw *rc2);
void updateCurrDrive();
void updateCurrHitchPose();
void updateRoboClaw(int velMsg, int steerMsg, int hitchMsg);
char checkUserInput();
int computeHitchMsg();
int steerAckToCmd(float ack_steer);
int velAckToCmd(float ack_vel);
float mapPrecise(float x, float inMin, float inMax, float outMin, float outMax);

#endif
