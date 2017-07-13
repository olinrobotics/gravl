#include <Arduino.h>

//Arduino Libraries
#include <Encoder.h>

//ROS library and messages
#include "ros.h"
#include "std_msgs/Bool.h"
#include "ackermann_msgs/AckermannDrive.h"

//User classes
#include "config.h"
#include "estop.h"
#include "lights.h"
#include "MotionControl.h"

// ROS variables
ros::NodeHandle nh;
Estop *e;
MotionControl *m;
Lights l;

Encoder left(11,12);
Encoder right(9,10);
unsigned long lastmillis = millis();
unsigned char mode = 4;

void setup() {
  nh.getHardware()->setBaud(ROSSERIAL_BAUD);
  nh.initNode(); // Initialize ROS nodehandle
  e->setup(&nh);
  e->onStop(stop);
  e->offStop(restart);
  m->setup(&nh);
  l.setup();
  pinMode(13, OUTPUT);
}

void loop() {
  if(mode != 0){
    if(nh.connected()){
      mode = 5;
    }
    else{
      mode = 4;
    }
  }
  if (millis() - lastmillis >= 100){
    l.send(1, mode);
    l.send(2, mode);
    nh.spinOnce();
    lastmillis = millis();
  }
  /*if (millis() - lastmillis == 100){
    //USB_SER.print(left.read());
    left.write(0);

    //USB_SER.print("       ");

    //USB_SER.println(right.read());
    lastmillis = millis();
    right.write(0);
  }*/
}

void stop(){
  mode = 0;
}

void restart(){
  mode = 3;
}
