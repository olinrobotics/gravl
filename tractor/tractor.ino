#include <Kangaroo.h>
#include "ros.h"
#include "config.h"
#include "std_msgs/Bool.h"
#include "estop.h"

// ROS variables
ros::NodeHandle nh;
Estop *e;

// Variables for the Kangaroo
KangarooSerial k(Serial2);
KangarooChannel steering(k, '1');
KangarooChannel gas(k, '2');


void setup() {
  Serial.begin(9600); // USB com
  Serial2.begin(9600); // Kangaroo com
  Serial3.begin(9600); // Neopixel com
  setupROS();
  setupKangaroo();
  e->setup();
}

void loop() {
  // put your main code here, to run repeatedly:

}

void setupROS(){
  nh.initNode();
}

void setupKangaroo(){
  steering.start();
  gas.start();
}

