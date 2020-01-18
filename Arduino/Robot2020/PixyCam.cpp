#include "PixyCam.h"

static Pixy2 PixyCam::pixy;
static PIDLoop PixyCam::pan = PIDLoop(400, 0, 400, true);
static PIDLoop PixyCam::tilt = PIDLoop(500, 0, 500, true);
static bool PixyCam::targetInView;
static int PixyCam::xValue;
static int PixyCam::distance;

static void PixyCam::initialize() {
  pixy.init();
  pixy.changeProg("color");
  pixy.setCameraBrightness(50);
  reset();
}

static void PixyCam::refresh() {
  // get block data
  pixy.ccc.getBlocks(1);
  if (pixy.ccc.numBlocks) {
    if (!targetInView)
      targetInView = true;
    // update values
    xValue = pixy.ccc.blocks[0].m_x;
    int width = pixy.ccc.blocks[0].m_width;
    byte angle = (double)width / (double)pixy.frameWidth * 60.0 / 2.0;
    distance = 3.5 * sin((90 - angle) / 2) * (1.0 / sin(angle));
  } else if (targetInView)
    targetInView = false;
}

static void PixyCam::reset() {
  // reset servo PID loops to set camera position to center
  pan.reset();
  tilt.reset();
  pixy.setServos(500, 850);
}

static bool PixyCam::getTargetInView() {
  return targetInView;
}

static int PixyCam::getXValue() {
  return xValue;
}

static int PixyCam::getDistance() {
  return distance;
}
