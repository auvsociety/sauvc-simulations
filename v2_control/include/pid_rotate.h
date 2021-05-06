#ifndef PID_ROTATE
#define PID_ROTATE

#include <cmath>

/**
 * @brief PID controller class for rotational motion of AUV_v2.
 */

class PidRotate {
private:
  double prevErr_;
  double output_;
  double p_, i_, d_;
  
  double f_round(double f, int decimals);

public:
  PidRotate();

  double kp_;
  double ki_;
  double kd_;
  double snstvty_;  // set point +- range

  double update(double set_point, double cur_state, double dt);
  void reset();
};

#endif // PID_ROTATE
