#ifndef I2C_h
#define I2C_h

#include "Arduino.h"
#include <Wire.h>

class I2C {
  public:
    static void initialize(byte address);
    static void receiveEvent();
    static void requestEvent();
    static byte getPattern();
    static void setWriteData(bool targetInView, int xValue, byte distance);
    
  private:
    // data received from RoboRio
    static byte readData[1];
    // data to send to RoboRio
    static byte writeData[5];

    static void splitValue(int value, byte startIndex, byte endIndex);
};

#endif
