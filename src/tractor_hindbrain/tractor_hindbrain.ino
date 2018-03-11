/**********************************************************************
 * KUBO Hindbrain Code (Teensy 3.5)
 * @file hind_brain.ino
 * @author: Connor Novak
 * @author: Carl Moser
 * @email: connor.novak@students.olin.edu
 * @version: 1.3
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
const byte ROBOCLAW_UPDATE_RATE = 500;


// Def/Init Global Variables ----------V----------V----------V
boolean isEStopped = false;
unsigned long prevMillis = millis();


// Declare ROS stuff
ros::NodeHandle nh;
OAKEstop *e_stop;
OAKSoftSwitch *autonomous_light;
OAKEncoder *left_encoder;
OAKEncoder *right_encoder;

/*
 * FUNCTION: setup()
 * DESC: runs once on startup
 * ARGS: none
 * RTNS: none
 */
void setup() { // ----------S----------S----------S----------S----------S

  // Set up ROS node and initialize subscriber
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
}


/*
 * FUNCTION: loop()
 * DESC:  loops constantly
 * ARGS: none
 * RTNS: none
*/
void loop() { // ----------L----------L----------L----------L----------L
  // Check if connected | Estop if not connected
  if(nh.connected()){
  
    // Sends commands to RoboClaw every ROBOCLAW_UPDATE_RATE milliseconds
    if (millis() - prevMillis > ROBOCLAW_UPDATE_RATE && !isEStopped) {
      //updateRoboClaw(velMsg, steerMsg);
    }
    
    // Updates node
    nh.spinOnce();
    delay(1);
  } else{
    eStop();
  }
}


// ----------F----------F----------F----------F----------F----------F----------F

/*
 *  FUNCTION eStop()
 *  DESC: Estops tractor, sends error message, flips estop state
 *  ARGS: none
 *  RTRNS: none
 */
void eStop() {

  isEStopped = true;

  // Logs estop msg
  char i[32];
  snprintf(i, sizeof(i), "ERR: Tractor E-Stopped");
  nh.loginfo(i);

  // Toggle relay to stop engine
  digitalWrite(ESTOP_PIN, HIGH);
}


/*
 * FUNCTION eStart()
 * DESC: Changes estopped state upon tractor restart
 * ARGS: none
 * RTNS: none
 */
 void eStart() {
  isEStopped = false;

  // Logs verification msg
  char i[32];
  snprintf(i, sizeof(i), "MSG: EStop Disactivated");
  nh.loginfo(i);

  digitalWrite(ESTOP_PIN, LOW);
 }
