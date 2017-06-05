#ifndef ESTOP_H
#define ESTOP_H

#include "ros.h"
#include "std_msgs/Bool.h"
#include "config.h"

class Estop{
public:
	static void setup();
	
private:
  static ros::Publisher *eStop;
  static std_msgs::Bool stopped;
  static void onChange();
};

#endif //ESTOP_H
