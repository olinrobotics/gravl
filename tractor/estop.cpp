#include "estop.h"

ros::Publisher *Estop::eStop;
std_msgs::Bool Estop::stopped;

void Estop::setup(){
  eStop = new ros::Publisher("/estop", &stopped);
  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), onChange, CHANGE);
  
}

void Estop::onChange(){
  stopped.data = !stopped.data;
  eStop->publish(&stopped);
}
