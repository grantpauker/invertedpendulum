#include "pid.hpp"
#include <math.h>

#define CLAMP(x, min, max) (((x) > (max)) ? (max) : (((x) < (min)) ? (min) : (x)))
PID::PID(float _kp, float _ki, float _kd, float _max_out, float _min_out)
{
  kp = _kp;
  ki = _ki;
  kd = _kd;

  setpoint = 0;
  error = 0;
  error_last = 0;
  integral = 0;

  min_out = _min_out;
  max_out = _max_out;
}
void PID::setSetpoint(float _setpoint)
{
  setpoint = _setpoint;
}

float PID::getError(void)
{
  return error;
}

float PID::calculate(float input, float dt)
{
  if (dt < 1e-6)
  {
    dt = 1e-6;
  }
  error = setpoint - input;
  float proportion = error;
  integral += error * dt;
  float derivative = (error - error_last) / dt;
  float output = (kp * proportion) + (ki * integral) + (kd * derivative);
  return CLAMP(output, min_out, max_out);
}
void PID::reset(void)
{
  error = 0;
  error_last = 0;
  integral = 0;
  setpoint = 0;
}