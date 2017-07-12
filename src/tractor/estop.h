#ifndef ESTOP_H
#define ESTOP_H

/******************************************************************************
 * Estop class for OAK (Olin Autonomous Kore)
 * @author: Carl Moser
 * @email: carl.moser@students.olin.edu
 ******************************************************************************/

#include "ros.h"
#include "std_msgs/Bool.h"
#include "config.h"

#ifndef ESTOP_PIN
	#error THE ESTOP PIN NEEDS TO BE DEFINED
#endif

#ifndef ESTOP_DEBOUNCE_TIME
 #pragma THE ESTOP DEBOUNCE TIME WAS NOT DEFINED - DEFAULTING TO 1
 #define ESTOP_DEBOUNCE_TIME 1 // half-milliseconds
#endif

class Estop{
public:
	/*
	 * The only functions users should be concerned with
	 */
	static void setup(ros::NodeHandle *nh);
	static void onStop(void (*func)());
	static void offStop(void (*func)());

private:
  static ros::Publisher *hardEStop;
	static ros::Subscriber<std_msgs::Bool> *softEStop;
  static std_msgs::Bool stopped;
	static bool softStopped;
	static long last_mill;
	static void (*stopfunc)();
	static void (*startfunc)();
  static void onChange();
	static void softStopCB(const std_msgs::Bool &message);
	static void defaultOnStop();
	static void defaultOffStop();
};

#endif //ESTOP_H
