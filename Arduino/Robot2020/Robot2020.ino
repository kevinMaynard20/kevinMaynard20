// Robot 2020
// Arduino code for Team 20's 2020 FIRST Robotics Competition Robot

#include "I2C.h"
#include "PixyCam.h"

void setup() {
  Serial.begin(9600);
  I2C::initialize(0x1);
  LEDStrip::initialize(6, 144, 15);
  PixyCam::initialize();
}

void loop() {
  LEDStrip::refresh(I2C::getPattern());
  PixyCam::refresh();
  I2C::setWriteData(PixyCam::getTargetInView(), PixyCam::getXValue(), PixyCam::getDistance());
}
