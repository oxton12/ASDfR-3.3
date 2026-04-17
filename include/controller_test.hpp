//==============================================================================
// Authors : Max Solis, Aleksei Obshatko
// Group : ASDfR 5
// License : LGPL open source license
//
// Brief : Finite State Machine code for controlling RELBot. Based
// on the provided templates
//==============================================================================

#ifndef CONTROLLER_TEST_HPP
#define CONTROLLER_TEST_HPP

#include <evl/clock.h>
#include <math.h>

#include "LoopController.h"
#include "XenoFrt20Sim.hpp"

#pragma pack( \
    1)  // https://carlosvin.github.io/langs/en/posts/cpp-pragma-pack/#_performance_test
struct ThisIsAStruct {
  int this_is_a_int = 0;
  double this_is_a_double = 100.0;
  float this_is_a_float = 10.0;
  char this_is_a_char = 'R';
  bool this_is_a_bool = false;
};

#pragma pack(0)

class ControllerTest : public XenoFrt20Sim {
 public:
  ControllerTest(uint write_decimator_freq, uint monitor_freq);
  ~ControllerTest();

 private:
  XenoFileHandler file;
  struct ThisIsAStruct data_to_be_logged;
  LoopController controller;

  double u[4];
  double y[2];

  int encMax = 16383;          // Maximal value of encoder reading
  int turnCount = 4 * encMax;  // Amount of encoder counts per one rotation
  int prevEncLeft = 0;         // Previous reading of the left encoder
  int prevEncRight = 0;        // Previous reading of the right encoder
  int PWMMax = 2047;           // Max value of PWM

  double diameter = 0.101;   // Wheel diameter in m
  double wheelbase = 0.209;  // Distance between wheels
  double theta = 0;          // Orientation of the robot
  double xPos = 0;           // X position in m
  double yPos = 0;           // Y position in m

 protected:
  // Functions
  int initialising() override;
  int initialised() override;
  int run() override;
  int stopping() override;
  int stopped() override;
  int pausing() override;
  int paused() override;
  int error() override;

  // current error
  int current_error = 0;
};

#endif  // CONTROLLER_TEST_HPP
