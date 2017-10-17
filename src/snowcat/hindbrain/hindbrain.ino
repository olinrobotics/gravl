/**********************************************************************
   Snowcat Hindbrain Code (UNO Rev 3)
   @file hindbrain.ino
   @author: Connor Novak
   @email: connor.novak@students.olin.edu
   @version: 1.0

   Basic control of drive motors and plow over XBee Radio
 **********************************************************************/

// Include Libraries
#include <Arduino.h>
#include "RoboClaw.h"
#include "hindbrain.h"
#include <Servo.h>

// Declare & Initialize Pins

const byte LED_PIN = 13;
const byte PLOW_SERVO_PIN = 3;
const byte LEFT_IR_PIN = A0;
const byte RIGHT_IR_PIN = A1;
Servo plowServo;

// Declare variables
int plowPos = 180;
int leftDist;
int rightDist;
long ledTimer = millis();     //ms
boolean ledState = LOW;

// Define roboclaw variables
#define RC_SERIAL Serial1
#define address 0x80
const int ROBOCLAW_UPDATE_RATE = 500;
RoboClaw roboClaw(&Serial1, 10000);


void setup() {

  // Set pinmodes
  plowServo.attach(PLOW_SERVO_PIN);
  pinMode(LED_PIN, OUTPUT);

  // Open Serial connection(s)
  // Serial.begin(9600);
  roboClaw.begin(38400);

  // Set actuators to default positions
  roboClaw.SpeedM1(address,12000);
  roboClaw.SpeedM2(address, 12000);
}

void loop() {

  // ---------- Sense ----------

  leftDist = analogRead(LEFT_IR_PIN);
  rightDist = analogRead(RIGHT_IR_PIN);
  // ---------- Think ----------

  // Check value ranges
  plowPos = checkPlow(plowPos);

  // Update LED
  ledState = updateLED(&ledTimer, ledState);


  // ---------- Act ----------

  // Send plow signal
  plowServo.write(plowPos);

  // Send LED signal
  digitalWrite(LED_PIN, ledState);
}

