#include "lights.h"
#include <Wire.h>

void Lights::setup(){
  Wire.begin();
}

unsigned char Lights::send(unsigned char address, unsigned char data){
  Wire.beginTransmission(address);
  Wire.send(data);
  return Wire.endTransmission();
}
