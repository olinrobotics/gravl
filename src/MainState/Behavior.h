/*
 * @file Behavior.h
 * @brief behavior function prototypes
 *
 * @author Connor Novak
 * @email connor@students.olin.edu
 */

#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include "gravl/TwistLabeled.h"

class Behavior {
  public:
    Behavior(const char* n, const int l);

  // Getters and Setters
    int getId();
    gravl::TwistLabeled getMessage();
    void setMessage(gravl::TwistLabeled msg);

  private:
    int id;
    const char* name;
    gravl::TwistLabeled message;

};

#endif //BEHAVIOR_H
