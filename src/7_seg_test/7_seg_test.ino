//Derived from https://www.instructables.com/id/7-Segment-Display-On-Arduino/

#include "lights.h"

SevenSegDisplay seg_display;

void setup() {               
 seg_display.init();
}


void loop() {
 for(int i=0;i<10;i++)
 {
   seg_display.displayDigit(i);
   delay(1000);
   seg_display.turnOff();
   delay(1000);
 }
}
