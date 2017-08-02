#ifndef BUTTON_H
#define BUTTON_H

#include <Time.h>
#include <TimeAlarms.h>
#include "ros.h"
#include "std_msgs/Bool.h"
#include "config.h"

static void dummyFunc() {return;}
extern void attachInterrupt2(uint8_t pin, void (*function)(void*), int mode, void* clas);

class button{
public:
	explicit button(ros::NodeHandle *nh, const char* name, const int pin, const unsigned int debounceTime, const int trigger);
	void onPress(void (*func)());
	void offPress(void (*func)());
	static void globalPress(void* instance);

private:
  ros::Publisher *but;
  std_msgs::Bool pressed;
	const unsigned int debounceTime;
	const int pin;
	long last_mill;
  void onChange();
  void (*pressedfunc)() = dummyFunc;
  void (*releasedfunc)() = dummyFunc;
};

#endif //BUTTON_H
