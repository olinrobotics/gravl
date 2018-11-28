/*
 * @file Behavior.cpp
 * @author Connor Novak
 * @date 2018-11-21
 *
 * Behavior class with index
 */

#include "Behavior.h"

Behavior::Behavior(const char* n, const int l) {
   Behavior::name = n;
   Behavior::id = l;
 }

 Behavior::Behavior(int l) {
   name = "hello";
   id = l;
 }

int Behavior::getId() {
  return id;
}
