#pragma once

#include "arm_math_types.h"

struct quat_t {
  // Technically undefined
  union {
    struct {
      float32_t w;
      float32_t x;
      float32_t y;
      float32_t z;
    };
    float32_t data[4];
  };

  quat_t &operator=(quat_t const &rhs);

  quat_t &operator+=(quat_t const &rhs);

  quat_t &operator*=(float32_t rhs);

  quat_t &operator*=(quat_t const &rhs);

  quat_t &operator/=(float32_t rhs);

  quat_t &normalize();
};

quat_t operator+(quat_t const &lhs, quat_t const &rhs);

quat_t operator*(quat_t const &lhs, float32_t rhs);

quat_t operator*(float32_t lhs, quat_t const &rhs);

quat_t operator*(quat_t const &lhs, quat_t const &rhs);

quat_t operator/(quat_t const &lhs, float32_t rhs);
