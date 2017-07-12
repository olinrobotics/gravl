#include "estop.h"

/******************************************************************************
 * Estop class for OAK (Olin Autonomous Kore)
 * @author: Carl Moser
 * @email: carl.moser@students.olin.edu
 *
 * This is meant to be a modular class for any robot within the lab
 * it automatically creates a subscriber and publisher for estop(s)
 * true is stop
 *
 * TODO: add support for multiple estop pins
 * TODO: check the state of the pin(s) on change
 *
 * TO USE:
 *  1- in the config header #define ESTOP_PIN and ESTOP_DEBOUNCE_TIME
 *  2- include the header in the .ino file (#include "estop.h")
 *  3- declare a pointer to the class (Estop *e)
 *  4- within void setup do the following:
 *    a- call the setup function with the memory address of the nodehandle
 *       (e->setup(&nh))
 *    b- set the function you want to run on a stop (e->onStop(stop)) where
 *       stop is a function
 *    c- set the function you want to run after a stop (e->offStop(stop)) where
 *       restart is a function
 *  5- that is it...the class will take care of the rest
 ******************************************************************************/

/*
 * Function and variable declarations
 * Needed since the class is static
 */
ros::Publisher *Estop::hardEStop;
ros::Subscriber<std_msgs::Bool> *Estop::softEStop;
std_msgs::Bool Estop::stopped;
bool Estop::softStopped = false;
long Estop::last_mill;
void Estop::defaultOnStop(){};
void Estop::defaultOffStop(){};
void (*Estop::stopfunc)() = defaultOnStop;
void (*Estop::startfunc)() = defaultOffStop;

/*
 * Setup function for the class
 *
 * Initializes a publisher and subsciber
 * attaches the estop pin
 */
void Estop::setup(ros::NodeHandle *nh){
  hardEStop = new ros::Publisher("/hardestop", &stopped);
  softEStop = new ros::Subscriber<std_msgs::Bool>("/softestop", &softStopCB);
  nh->advertise(*hardEStop);
  nh->subscribe(*softEStop);
  last_mill = millis();
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), onChange, CHANGE);
  stopped.data = !digitalRead(ESTOP_PIN);
}

/*
 * Function that runs on a hardware estop
 *
 * If stopping - calls the stopfunc and publishes true
 * If reseting from stop - calls the startfunc and publishes false
 */
void Estop::onChange(){
  if(millis()-last_mill >= ESTOP_DEBOUNCE_TIME*50){
    stopped.data = !stopped.data;
    hardEStop->publish(&stopped);
    if(stopped.data){
      (*Estop::stopfunc)();
    }
    else{
      if(!softStopped)
        (*Estop::startfunc)();
    }
    last_mill = millis();
  }
}

/*
 *Function that runs on sofware estop
 */
void Estop::softStopCB(const std_msgs::Bool &message){
  if(message.data){
    (*Estop::stopfunc)();
    softStopped = true;
  }
  else{
    softStopped = false;
    if(!stopped.data)
      (*Estop::startfunc)();
  }
}

/*
 * Set stop function pointer to function pointer that is passed in
 */
void Estop::onStop(void (*func)()){
  stopfunc = func;
}

/*
 * Set restart function pointer to function pointer that is passed in
 */
void Estop::offStop(void (*func)()){
  startfunc = func;
}
