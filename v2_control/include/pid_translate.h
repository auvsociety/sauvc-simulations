#ifndef PID_TRANSLATE_H
#define PID_TRANSLATE_H

#include <cmath>
// #include <iostream>     // for debugging

/**
 * @brief PID controller class for translation motion of AUV_v2.
 */

class PidTranslate
{
private:
  double prevErr_;
  double output_;
  double p_, i_, d_;
  
  double f_round(double f, int decimals);

public:
  PidTranslate();

  double kp_;
  double ki_;
  double kd_;
  double snstvty_;  // set point +- range

  double update(double set_point, double cur_state, double dt);
  void reset();
};

#endif  // PID_TRANSLATE_H
