#include "estop.h"

/******************************************************************************
 * Estop class for OAK (Olin Autonomous Kore)
 * @file estop.cpp
 * @author Carl Moser
 * @email carl.moser@students.olin.edu
 * @version     1.1
 *
 * This is meant to be a modular class for any robot within the lab
 * it automatically creates a publisher and subscriber for an estop
 *
 * @class Estop estop.h "estop.h"
 ******************************************************************************/

/*
 * Constructor for the class
 *
 * Initializes a publisher and subsciber
 * attaches the estop pin
 */
Estop::Estop(ros::NodeHandle *nh, const int pin, const unsigned int debounceTime):pin(pin),debounceTime(debounceTime){
  hardEStop = new ros::Publisher("/hardestop", &stopped);
  softEStop = new ros::Subscriber<std_msgs::Bool, Estop>("/softestop", &Estop::softStopCB, this);
  nh->advertise(*hardEStop);
  nh->subscribe(*softEStop);
  last_mill = millis();
  pinMode(pin, INPUT_PULLUP);
  attachInterrupt2(digitalPinToInterrupt(pin), &Estop::globalStop, CHANGE, this);
  stopped.data = !digitalRead(pin);
}

/*
 * Global function that calls the object function
 *
 * @param[in] instance Instance of the class
 */
void Estop::globalStop(void *instance){
  static_cast<Estop*>(instance)->onChange();
}

/*
 * Function that checks if the system is estopped
 *
 * @return True if stopped
 */
bool Estop::isStopped(){
  return softStopped|stopped.data;
}

/*
 * Function that runs on a hardware estop
 *
 * If stopping - calls the stopfunc and publishes true
 * If reseting from stop - calls the startfunc and publishes false
 */
void Estop::onChange(){
  if(millis()-last_mill >= debounceTime*50){
    /*if(digitalRead(ESTOP_PIN)){
      stopped.data = true;
      hardEStop->publish(&stopped);
      (*Estop::stopfunc)();
    }
    else{
      stopped.data = false;
      hardEStop->publish(&stopped);
      (*Estop::startfunc)();
    }*/
    stopped.data = !stopped.data;
    hardEStop->publish(&stopped);
    if(stopped.data){
      (*stopfunc)();
    }
    else{
      if(!softStopped)
        (*startfunc)();
    }
    last_mill = millis();
  }
}

/*
 *Function that runs on sofware estop
 */
void Estop::softStopCB(const std_msgs::Bool &message){
  if(message.data){
    (*stopfunc)();
    softStopped = true;
  }
  else{
    softStopped = false;
    (*startfunc)();
    //if(!stopped.data)
      //(*startfunc)();
  }
}

void Estop::onStop(void (*func)()){
  stopfunc = func;
}

void Estop::offStop(void (*func)()){
  startfunc = func;
}
