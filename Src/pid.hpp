#pragma once

#include "arm_math_types.h"

class PID {
private:
  float32_t Kp;
  float32_t Ti;
  float32_t Td;
  float32_t N;
  float32_t Ts;

  float32_t error;
  float32_t prv_error = 0;
  float32_t integral = 0;
  float32_t prv_derivative = 0;
  float32_t int_derivative = 0;
  float32_t derivative;
  float32_t proportional;

public:
  PID(float32_t Kp, float32_t Ti, float32_t Td, float32_t N, float32_t Ts);

  float32_t update(float32_t setpoint, float32_t feedback);
};
