/*
  @file HindBrain.h
  Purpose: Config file for HindBrain.ino
*/

#ifndef HIND_BRAIN_H
#define HIND_BRAIN_H

#define USE_USBCON

// Libraries
#include <Arduino.h>                        // Microcontroller standard API
#include <Encoder.h>                        // Hitch height encoder lib

#include <ros.h>                            // rosserial library
#include <std_msgs/Empty.h>                 // Watchdog hf connection monitor
#include <std_msgs/String.h>                // rosserial String msg
#include <geometry_msgs/Pose.h>             // rosserial Hitch msg

#include "Estop.h"                          // OAK Estop
#include "SoftSwitch.h"                     // OAK SoftSwitch
#include "RoboClaw.h"                       // Motor controller API
#include "ackermann_msgs/AckermannDrive.h"  // rosserial Steering msg

// Arduino Pins
const byte ESTOP_SENSE_PIN = 1;
const byte ESTOP_RELAY_PIN = 2;
const byte AUTO_LIGHT_PIN = 3;
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
const int VEL_CMD_MIN = 1200;   // Roboclaw cmd for min speed
const int VEL_CMD_STOP = 1029;  // Roboclaw cmd for stopped
const int VEL_CMD_MAX = 850;    // Roboclaw cmd for max speed
const int VEL_MSG_MIN = -2;     // Ackermann msg min speed
const int VEL_MSG_MAX = 2;      // Ackermann msg max speed

// Steering Motor Ranges
const int STEER_CMD_LEFT = 500;   // Roboclaw cmd for max left turn
const int STEER_CMD_CENTER = 975; // Roboclaw cmd for straight
const int STEER_CMD_RIGHT = 1275; // Roboclaw cmd for max right turn
const int STEER_MSG_LEFT = 45;    // Ackermann msg min steering angle
const int STEER_MSG_CENTER = 0;   // Ackermann msg center steering angle
const int STEER_MSG_RIGHT = -30;  // Ackermann msg max steering angle

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
void checkUserInput();
void eStartTractor();
void eStopTractor();
void runStartupSequence();
void stopEngine();
void stopRoboClaw(RoboClaw *rc1, RoboClaw *rc2);
void updateCurrDrive();
void updateCurrHitchPose();
void updateRoboClaw(int velMsg, int steerMsg, int hitchMsg);
int computeHitchMsg();
int steerAckToCmd(float ack_steer);
int velAckToCmd(float ack_vel);
float mapPrecise(float x, float inMin, float inMax, float outMin, float outMax);

#endif
