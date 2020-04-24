#ifndef PID_H
#define PID_H

#include "Arduino.h"

class PID
{
public:
  PID(float _kp, float _ki, float _kd) : PID(_kp, _ki, _kd, -INFINITY, INFINITY) {}
  PID(float, float, float, float, float);

  void setSetpoint(float);
  float getError(void);
  float calculate(float, float);
  void reset(void);

private:
  float kp;
  float ki;
  float kd;

  float setpoint;

  float error;
  float error_last;

  float integral;

  float max_out;
  float min_out;
};

#endif