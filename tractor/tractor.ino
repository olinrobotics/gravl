
//Arduino Libraries
#include <Kangaroo.h>
#include <Encoder.h>

//ROS library and messages
#include "ros.h"
#include "std_msgs/Bool.h"

//User classes
#include "config.h"
#include "estop.h"
#include "MotionControl.h"

// ROS variables
ros::NodeHandle nh;
Estop *e;
MotionControl m;

void setup() {
  USB_SER.begin(USB_SPEED); // USB com
  nh.initNode(); // Initialize ROS nodehandle 
  e->setup(&nh);
  m.setup(&nh);
}

void loop() {
  // put your main code here, to run repeatedly:

}

