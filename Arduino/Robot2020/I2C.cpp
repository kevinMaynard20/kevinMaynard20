#include "I2C.h"

static byte I2C::readData[1];
static byte I2C::writeData[7];

static void I2C::initialize(byte address) {
  Wire.begin(address);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  for (byte i = 0; i < sizeof(readData) / sizeof(byte); i++)
    readData[i] = 0;
  for (byte i = 0; i < sizeof(writeData) / sizeof(byte); i++)
    writeData[i] = 0;
}

static void I2C::receiveEvent() {
  for (byte i = 0; Wire.available() > 0 && i < sizeof(readData) / sizeof(byte); i++)
    readData[i] = Wire.read();
}

static void I2C::requestEvent() {
  Wire.write(writeData, sizeof(writeData) / sizeof(byte));
}

static void I2C::setWriteData(bool targetInView, int xValue, int distance) {
  writeData[0] = targetInView ? 1 : 0;
  splitValue(xValue, 1, 3);
  splitValue(distance, 4, 6);

//  if (xValue <= 127) {
//    writeData[1] = xValue;
//    writeData[2] = 0;
//    writeData[3] = 0;
//  } else {
//    writeData[1] = 127;
//    if (xValue <= 255) {
//      writeData[2] = xValue - 127;
//      writeData[3] = 0;
//    } else {
//      writeData[2] = 127;
//      writeData[3] = xValue - 254;
//    }
//  }
//
//  if (distance <= 127) {
//    writeData[4] = distance;
//    writeData[5] = 0;
//    writeData[6] = 0;
//  } else {
//    writeData[4] = 127;
//    if (distance <= 255) {
//      writeData[5] = distance - 127;
//      writeData[6] = 0;
//    } else {
//      writeData[5] = 127;
//      writeData[6] = distance - 254;
//    }
//  }
}

static void I2C::splitValue(int value, byte startIndex, byte endIndex) {
  for (byte i = startIndex; i <= endIndex; i++) {
    if (value <= 127) {
      writeData[i] = value;
      for (byte j = i + 1; j <= endIndex; j++)
        writeData[j] = 0;
    } else {
      writeData[i] = 127;
      value -= 127;
    }
  }
}
