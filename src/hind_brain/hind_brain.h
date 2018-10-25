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

  // Ranges
  

#endif
