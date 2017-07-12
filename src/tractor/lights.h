#ifndef LIGTHS_H
#define LIGHTS_H

#include "config.h"

class Lights{
public:
  void setup();
  unsigned char send(unsigned char address, unsigned char data);
private:
  unsigned char lights[NUM_LIGHTS];
};

#endif //LIGHTS_H
