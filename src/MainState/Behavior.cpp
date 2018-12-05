/*
 * @file Behavior.cpp
 * @brief behavior class correlating names, ids, priorities
 *
 * @author Connor Novak
 * @email connor@students.olin.edu
 * @date 2018-11-21
 * @version 1.0.0
 */

#include "Behavior.h"

/*
 * @brief Constructor TODO(connor@students) Turn into struct
 */
Behavior::Behavior(const char* name, const int label)
 : name(name), id(label) { }

int Behavior::getId() {
  return id;
}

gravl::TwistLabeled Behavior::getMessage() {
  return message;
}

void Behavior::setMessage(gravl::TwistLabeled msg) {
  //TODO(connor@students): Check that label is equal to id
  message = msg;
}
