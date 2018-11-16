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
#include "estop.h"                          // Used to implement estop class
#include "soft_switch.h"                    // Used to implement auto switch

// Arduino Pins
const byte AUTO_LED_PIN = 3;
const byte ESTOP_PIN = 2;

// Roboclaw Constants
#define RC1_ADDRESS 0x80
#define RC2_ADDRESS 0x80
// Timeout below equivalent to 10 ms
#define RC_TIMEOUT 10000

// General Constants
#define DEBUG True

// Velocity Motor Ranges
const int VEL_CMD_MIN = 1400;       // Roboclaw cmd for max speed
const int VEL_CMD_MAX = 0;          // Roboclaw cmd for min speed
const int VEL_MSG_MIN = 0;          // Ackermann msg min speed
const int VEL_MSG_MAX = 2;          // Ackermann msg max speed

// Steering Motor Ranges
const int STEER_CMD_LEFT =   500;   // Roboclaw cmd for max left turn
const int STEER_CMD_CENTER = 975;   // Roboclaw cmd for straight
const int STEER_CMD_RIGHT =  1275;  // Roboclaw cmd for max right turn
const int STEER_MSG_LEFT =   45;    // Ackermann msg min steering angle
const int STEER_MSG_CENTER = 0;     // Ackermann msg center steering angle
const int STEER_MSG_RIGHT = -30;    // Ackermann msg max steering angle

// function prototypes

void ackermannCB(const ackermann_msgs::AckermannDrive&);
void waitForConnection(bool estop);
void stopEngine();
void eStop();
void eStart();

#endif
