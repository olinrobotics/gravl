/**********************************************************************
 * Snowcat Code (Teensy 3.5)
 * @file snowcatv2.ino
 * @author: Carl Moser
 * @email: carl.moser@students.olin.edu
 * @version: 1.0
 * 
 * SnowCat code updated to use the OAK architecture
 * This is the first "full" test of OAK
 *
 **********************************************************************/

// Include Libraries
#include <Arduino.h>
#include <OAKSoftSwitch.h>
#include <OAKVL53.h>
#include <OAKServo.h>
#include "motion.h"

// Motion
#define RC_SERIAL Serial1
#define RC_BAUD 38400
#define RC_ADDRESS 0x80


// TOF array
#define NUM_SENSOR 3
#define LEFT_TOF 1
#define CENTER_TOF 2
#define RIGHT_TOF 3

// Plow
#define PLOW_SERVO_PIN 4

// Lights
#define LIGHT_PIN 5

// OAK
OAKServo *plow;
OAKSoftSwitch *light;
OAKVL53 *tof_array[NUM_SENSOR];
const String names[] = {"Left", "Center", "Right"};
const byte pins[] = {LEFT_TOF, CENTER_TOF, RIGHT_TOF};
const byte addresses[] = {0x31,0x30,0x29};

// Variables
ros::NodeHandle nh;
Motion *motion;


void setup() {
  nh.initNode();

  /*******************************************
  * 
  * Setting up the TOF array
  *
  *******************************************/
  for(int i = 0; i < NUM_SENSOR; i++){
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], LOW);
  }
  delay(15); // Give the TOF sensors time to reset
  for(int i = 0; i < NUM_SENSOR; i++){
    digitalWrite(pins[i], HIGH);
    tof_array[i] = new OAKVL53(&nh, names[i].c_str(), 50, addresses[i]);
  }

  /*******************************************
  * 
  * Setting up other OAK stuff
  *
  *******************************************/
  plow = new OAKServo(&nh, "/plow", PLOW_SERVO_PIN);
  light = new OAKSoftSwitch(&nh, "/light", LIGHT_PIN);
  motion = new Motion(&nh, &RC_SERIAL, RC_BAUD, RC_ADDRESS);
}

void loop() {
  nh.spinOnce();
  for(int i = 0; i < NUM_SENSOR; i++){
    tof_array[i]->publish();
  }
}

