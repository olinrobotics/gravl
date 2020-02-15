#include "Estop.h"

/*
  @file         estop.cpp
  Class for OAK estop (Olin Autonomous Kore)
  @author       Carl Moser
  @maintainer   Olin GRAVL
  @email        olingravl@gmail.com
  @version      1.1
  @date         2020-02-14
  
  This is meant to be a modular class for any robot within the lab
  Attaches to pin and reads low as estop, publishes estops to /hardestop,
  estops based on msgs received on /softestop.
*/


Estop::Estop(ros::NodeHandle *nh, const int pin, const unsigned int debounceTime):pin(pin),debounceTime(debounceTime) {
  // Constructor - init publisher, subscriber, attach interrupt to pin.
  hardEStop = new ros::Publisher("/hardestop", &stopped);
  softEStop = new ros::Subscriber<std_msgs::Bool, Estop>("/softestop", &Estop::softStopCB, this);
  nh->advertise(*hardEStop);
  nh->subscribe(*softEStop);
  last_mill = millis();
  pinMode(pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin), &Estop::globalStop, CHANGE);
  stopped.data = !digitalRead(pin);
}


void Estop::globalStop(void *instance){
  // Global function that calls the object function
  static_cast<Estop*>(instance)->onChange();
}


bool Estop::isStopped(){
  return softStopped|stopped.data;
}


void Estop::onChange() {
  // Update state & call appropriate start|stop func on pin change.
  if (millis() - last_mill >= debounceTime) {
    if (digitalRead(pin)) {
      stopped.data = true;
      hardEStop->publish(&stopped);
      (*Estop::stopfunc)();
    } else {
      stopped.data = false;
      hardEStop->publish(&stopped);
      (*Estop::startfunc)();
    }
    stopped.data = !stopped.data;
    hardEStop->publish(&stopped);
    if (stopped.data) {
      (*stopfunc)();
    } else {
      if (!softStopped)
        (*startfunc)();
    }
    last_mill = millis();
  }
}


void Estop::softStopCB(const std_msgs::Bool &message) {
  // Update state & call appropriate start|stop function on msg receipt.
  if (message.data) {
    (*stopfunc)();
    softStopped = true;
  } else {
    (*startfunc)();
    softStopped = false;
  }
}


void Estop::onStop(void (*func)()){
  stopfunc = func;
}


void Estop::offStop(void (*func)()){
  startfunc = func;
}
