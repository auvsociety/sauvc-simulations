#include "pid_translate.h"

PidTranslate::PidTranslate()
{
  kp_ = 10.0;
  kd_ = 5.0;
  ki_ = 0.01;
  snstvty_ = 0.002;
  p_ = 0;
  i_ = 0;
  d_ = 0;
}

double PidTranslate::update(double set_point, double cur_state, double dt)
{
  cur_state = f_round(cur_state, 3);

  // update proportional, differential and integral errors
  p_ = set_point - cur_state;  // current error
  if (fabs(p_) <= snstvty_)
    p_ = 0.0;
  i_ = i_ + dt * p_;          // i -> sum of prev errors
  d_ = (p_ - prevErr_) / dt;  // d -> rate of error

  // std::cout << "p: " << kp_ * p_ << std::endl;   // debugging

  prevErr_ = p_;

  // update control output
  output_ = kp_ * p_ + kd_ * d_ + ki_ * i_;

  return output_;
}

void PidTranslate::reset()
{
  p_ = i_ = d_ = output_ = prevErr_ = 0;
}

double PidTranslate::f_round(double f, int decimals)
{
  float value = (int)(f * pow(10, decimals) + .5);
  return (float)value / pow(10, decimals);
}
