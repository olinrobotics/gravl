#include "button.h"

/******************************************************************************
 * Button class for OAK (Olin Autonomous Kore)
 * @file button.cpp
 * @author Carl Moser
 * @email carl.moser@students.olin.edu
 * @version     1.0
 *
 * This is meant to be a modular class for any robot within the lab
 * it automatically creates a publisher for a button
 *
 * @class button button.h "button.h"
 *
 * @TODO update documentationS
 * @TODO look into reading pin state after a delay
 * @TODO optimize for other types of triggers
 ******************************************************************************/

/*
 * Setup function for the class
 *
 * Initializes a publisher with the given name
 * attaches the button pin
 * @param[in] nh Memory address of the main ros nodehandle
 * @param[in] name Name of the publisher
 * @param[in] pin Pin of the button
 * @param[in] debounceTime The debounce time for the button
 * @param[in] trigger When the interrupt should be triggered
 */
button::button(ros::NodeHandle *nh, const char* name, const int pin, const unsigned int debounceTime, const int trigger):pin(pin),debounceTime(debounceTime){
  but = new ros::Publisher(name, &pressed);
  nh->advertise(*but);
  last_mill = millis();
  pinMode(pin, INPUT_PULLUP);
  attachInterrupt2(digitalPinToInterrupt(pin), &button::globalPress, trigger, this);
  pressed.data = !digitalRead(pin);
}

/*
 * Global function that calls the object function
 * @param[in] instance Instance of the class
 */
void button::globalPress(void *instance){
  static_cast<button*>(instance)->onChange();
}

/*
 * Function that runs on interrupt
 *
 * If pressed - calls the pressedfunc and publishes true
 * If released - calls the releasedfunc and publishes false
 */
void button::onChange(){
  if(millis()-last_mill >= debounceTime){
    pressed.data = !pressed.data;
    but->publish(&pressed);
    if(pressed.data){
      (*pressedfunc)();
    }
    else{
      (*releasedfunc)();
    }
    last_mill = millis();
  }
}

/*
 * Set pressed function pointer to function pointer that is passed in
 * @param[in] func
 */
void button::onPress(void (*func)()){
  pressedfunc = func;
}

/*
 * Set released function pointer to function pointer that is passed in
 * @param[in] func Pointer to
 */
void button::offPress(void (*func)()){
  releasedfunc = func;
}
