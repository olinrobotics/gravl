/*
 * @file Behavior.cpp
 * @author Connor Novak
 * @date 2018-11-21
 *
 * Behavior class with index
 */

#include "Behavior.h"

Behavior::Behavior(const char* name, const int label)
 : name(name), id(label) {
   //message = gravl::TwistLabeled();
   //message.label = label;
 }

int Behavior::getId() {
  return id;
}

gravl::TwistLabeled Behavior::getMessage() {
  return message;
}

void Behavior::setMessage(gravl::TwistLabeled msg) {
  message = msg;
}
