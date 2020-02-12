#include "LEDStrip.h"

static Adafruit_NeoPixel LEDStrip::strip;

static void LEDStrip::initialize(byte pin, byte numLEDs, byte brightness) {
  strip = Adafruit_NeoPixel(numLEDs, pin, NEO_GRB + NEO_KHZ800);
  strip.begin();
  strip.setBrightness(brightness);
  strip.show();
}

static void LEDStrip::refresh(byte pattern) {
  // selects pattern to display
  switch (pattern) {
    // TODO: add a case for each pattern
  }
}

static void LEDStrip::allOff() {
  for (byte i = 0; i < strip.numPixels(); i ++)
    strip.setPixelColor(i, off);
}
