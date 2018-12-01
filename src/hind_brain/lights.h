
/******************************************************************************
 * @file        lights.h
 * Software header for interfacing with 7 segment display
 * @author      Amy Phung
 * @email       amy.phung@students.olin.edu
 * @version     1.0
 * @date        12/01/2018
 ******************************************************************************/

#ifndef LIGHTS_H
#define LIGHTS_H

#include <Arduino.h>

class SevenSegDisplay{
public:
  SevenSegDisplay();
  void init();
  void displayDigit(int);
  void turnOff();
  
private:
  int a = 2;  //For displaying segment "a"
  int b = 3;  //For displaying segment "b"
  int c = 4;  //For displaying segment "c"
  int d = 5;  //For displaying segment "d"
  int e = 6;  //For displaying segment "e"
  int f = 8;  //For displaying segment "f"
  int g = 9;  //For displaying segment "g"
};
#endif //LIGHTS_H
