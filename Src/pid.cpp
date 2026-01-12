#include "pid.hpp"

#include "arm_math_types.h"

PID::PID(float32_t Kp, float32_t Ti, float32_t Td, float32_t N, float32_t Ts)
    : Kp(Kp), Ti(Ti), Td(Td), N(N), Ts(Ts) {}

float32_t PID::update(float32_t setpoint, float32_t feedback) {
  error = setpoint - feedback;

  proportional = error;

  integral += 0.5 * Ts * (error + prv_error);

  derivative = N * (error - int_derivative);
  int_derivative += 0.5 * Ts * (derivative + prv_derivative);
  prv_derivative = derivative;

  prv_error = error;

  return Kp * (proportional + integral / Ti + derivative / Td);
}
