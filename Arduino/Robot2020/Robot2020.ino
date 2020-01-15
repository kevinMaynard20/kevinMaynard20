// Robot 2020
// Arduino code for Team 20's 2020 FIRST Robotics Competition Robot
// Andrew Sealing

#include "I2C.h"
#include "PixyCam.h"

void setup() {
  Serial.begin(9600);
  I2C::initialize(0x1);
  PixyCam::initialize();
}

void loop() {
  PixyCam::refresh();
  I2C::setWriteData(PixyCam::getTargetInView(), PixyCam::getXValue(), PixyCam::getWidth());
}
