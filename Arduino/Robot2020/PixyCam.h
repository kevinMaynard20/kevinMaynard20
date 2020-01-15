#ifndef PixyCam_h
#define PixyCam_h

#include "Arduino.h"
#include <Pixy2.h>
#include <PIDLoop.h>

class PixyCam {
  public:
    static void initialize();
    static void refresh();
    static bool getTargetInView();
    static int getXValue();
    static int getWidth();
    
  private:
    // Pixy2 object
    static Pixy2 pixy;
    // servo PID loops
    static PIDLoop pan;
    static PIDLoop tilt;
    // booleans for checking if blocks are detected
    static bool targetInView;
    // x-value of center of largest block
    static int xValue;
    // width of largest block
    static int width;
    
    static void reset();
};

#endif
