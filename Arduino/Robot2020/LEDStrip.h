#ifndef LEDStrip_h
#define LEDStrip_h

#include "Arduino.h"
#include <Adafruit_NeoPixel.h>

class LEDStrip {
  public:
    static void initialize(byte pin, byte numLEDs, byte brightness);
    static void refresh(byte pattern);

  private:
    // constant RGB values for different colors
    static const uint32_t off;
    static const uint32_t red;
    static const uint32_t orange;
    static const uint32_t yellow;
    static const uint32_t green;
    static const uint32_t blue;
    static const uint32_t purple;
    static const uint32_t white;
    // LED strip object
    static Adafruit_NeoPixel strip;
    
    // turns all LEDs off
    static void allOff();
};

#endif
