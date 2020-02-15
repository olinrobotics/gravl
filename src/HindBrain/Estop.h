/*
  @file       estop.h
  Software header for OAK estop (Olin Autonomous Kore)
  @author     Carl Moser
  @maintainer Olin GRAVL
  @email      olingravl@gmail.com
  @version    1.1
  @date       2020-02-14
*/

#ifndef ESTOP_H
#define ESTOP_H

#include "ros.h"
#include "std_msgs/Bool.h"

static void dummyFunc() {return;}

/*
  Class that attaches an estop to a rostopic.

  Usage:
    Define function to call on stop, function to call on start
    Instantiate class with nodehandle, hardware estop read pin, and debounce time in ms.
    
*/
class Estop {
  public:
    explicit Estop(ros::NodeHandle *nh, const int pin, const unsigned int debounceTime);
    static void globalStop(void* instance);
    void onStop(void (*func)());
    void offStop(void (*func)());
    bool isStopped();

  private:
    ros::Publisher *hardEStop;
    ros::Subscriber<std_msgs::Bool, Estop> *softEStop;
    std_msgs::Bool stopped;
    bool softStopped = false;
    const unsigned int debounceTime;
    const int pin;
    long last_mill;
    void (*stopfunc)() = dummyFunc;
    void (*startfunc)() = dummyFunc;
    void onChange();
    void softStopCB(const std_msgs::Bool &message);
};

#endif
