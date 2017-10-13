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
#include "hindbrain.h"
#include <Servo.h>

// Declare & Initialize Pins

byte LED_PIN = 13;
byte PLOW_SERVO_PIN = 3;
byte LEFT_IR_PIN = A0;
byte RIGHT_IR_PIN = A1;
Servo plowServo;

// Declare variables
int plowPos = 180;

int leftDist;
int rightDist;

long ledTimer = millis();     //ms
boolean ledState = LOW;


void setup() {

  // Set pinmodes
  plowServo.attach(PLOW_SERVO_PIN);
  pinMode(LED_PIN, OUTPUT);

  // Set actuators to default positions

  // Open Serial connection(s)
  Serial.begin(9600);
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

