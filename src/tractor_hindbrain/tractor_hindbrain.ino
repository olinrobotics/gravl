/**********************************************************************
 * KUBO Hindbrain Code (Teensy 3.5)
 * @file hind_brain.ino
 * @author: Connor Novak
 * @author: Carl Moser
 * @email: connor.novak@students.olin.edu
 * @version: 1.4
 *
 * Basic OAK_compatible control of velocity actuator and
 * steering actuator through ackermann steering messages
 * over /drive, autonomous activation through boolean
 * message over /auto, estop capability over /softestop
 **********************************************************************/

#include <Arduino.h>                        // Used for Arduino functions
#include "ros.h"                            // Used for rosserial communication
#include <OAKEstop.h>                       // Used to implement estop class
#include <OAKSoftSwitch.h>                  // Used to implement auto switch
#include <OAKEncoder.h>                     // Used to implement rotary encoders
#include "motion.h"                         // Used to control the motion actuators

#define DEBUG TRUE

// Init Constants ----------C----------C----------C

// Physical Pins
const byte AUTO_LIGHT_PIN = 3;
const byte ESTOP_PIN = 2;
const byte ESTOP_SENSE_PIN = 13;
const byte LEFT_ENCODER_A = 9;
const byte LEFT_ENCODER_B = 10;
const byte RIGHT_ENCODER_A = 11;
const byte RIGHT_ENCODER_B = 12;

// General Constants
const byte ENCODER_UPDATE_RATE = 100; // Milliseconds
const int ROBOCLAW_UPDATE_RATE = 500;


// Def/Init Global Variables ----------V----------V----------V
boolean is_e_stopped = false;
unsigned long prev_millis = millis();


// Declare ROS and Class stuff
ros::NodeHandle nh;
OAKEstop *e_stop;
OAKSoftSwitch *autonomous_light;
OAKEncoder *left_encoder;
OAKEncoder *right_encoder;
Motion *motion;


/*
 * FUNCTION: setup()
 * DESC: runs once on startup
 * ARGS: none
 * RTNS: none
 */
void setup() { // ----------S----------S----------S----------S----------S

  // Setup ROS
  nh.getHardware()->setBaud(115200);
  nh.initNode(); // Initialize ROS nodehandle

  // Setup E-stop stuff
  e_stop = new OAKEstop(&nh, ESTOP_SENSE_PIN, 1);
  e_stop->onStop(eStop);
  e_stop->offStop(eStart);
  pinMode(ESTOP_PIN, OUTPUT);

  // Setup the Auto light
  autonomous_light = new OAKSoftSwitch(&nh, "/auto", AUTO_LIGHT_PIN);

  // Setup encoders
  left_encoder = new OAKEncoder(&nh, "/left_encoder", ENCODER_UPDATE_RATE, LEFT_ENCODER_A, LEFT_ENCODER_B);
  right_encoder = new OAKEncoder(&nh, "/right_encoder", ENCODER_UPDATE_RATE, RIGHT_ENCODER_A, RIGHT_ENCODER_B);

  motion = new Motion(&nh);
  motion->homeActuators();
}


/*
 * FUNCTION: loop()
 * DESC:  loops constantly
 * ARGS: none
 * RTNS: none
*/
void loop() { // ----------L----------L----------L----------L----------L

  // Check if connected | Estop if not connected
  if (nh.connected()) {
  
    // Sends commands to RoboClaw every ROBOCLAW_UPDATE_RATE milliseconds
    if (millis() - prev_millis > ROBOCLAW_UPDATE_RATE && !is_e_stopped) {
      motion->updateRoboClaw();
    }

    // Publish the encoders
    left_encoder->publish();
    right_encoder->publish();

    // Updates node
    nh.spinOnce();
    delay(1);
  }
  else {
    eStop();
  }

  prev_millis = millis();
}


// ----------F----------F----------F----------F----------F----------F----------F

/*
 *  FUNCTION eStop()
 *  DESC: Estops tractor, sends error message, flips estop state
 *  ARGS: none
 *  RTRNS: none
 */
void eStop() {

  is_e_stopped = true;

  // Logs estop msg
  nh.logwarn("ERR: Tractor E-Stopped");

  // Enable relay to stop engine and home actuators
  digitalWrite(ESTOP_PIN, HIGH);
  motion->homeActuators();

  // Lock system for 2 seconds to make sure engine is killed
  delay(2000);
}


/*
 * FUNCTION eStart()
 * DESC: Changes estopped state upon tractor restart
 * ARGS: none
 * RTNS: none
 */
void eStart() {
  is_e_stopped = false;

  // Logs verification msg
  nh.loginfo("MSG: EStop Disactivated");

  digitalWrite(ESTOP_PIN, LOW);
}
