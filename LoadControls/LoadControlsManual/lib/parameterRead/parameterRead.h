/*
  Morse.h - Library for flashing Morse code.
  Created by David A. Mellis, November 2, 2007.
  Released into the public domain.
*/
#ifndef parameterRead_h
#define parameterRead_h

#include "Arduino.h"

class parameterRead
{
  public:
    parameterRead(int pin);
    void dot();
    void dash();
  private:
    int _pin;
};

#endif