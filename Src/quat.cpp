#include "quat.hpp"

#include "arm_math_types.h"
#include "dsp/quaternion_math_functions.h"

quat_t &quat_t::operator+=(quat_t const &rhs) {
  this->w += rhs.w;
  this->x += rhs.x;
  this->y += rhs.y;
  this->z += rhs.z;

  return (*this);
}

quat_t &quat_t::operator*=(quat_t const &rhs) {
  arm_quaternion_product_f32(this->data, rhs.data, this->data, 1);

  return (*this);
}

quat_t &quat_t::operator*=(float32_t rhs) {
  this->w *= rhs;
  this->x *= rhs;
  this->y *= rhs;
  this->z *= rhs;

  return (*this);
}

quat_t &quat_t::operator/=(float32_t rhs) {
  this->w /= rhs;
  this->x /= rhs;
  this->y /= rhs;
  this->z /= rhs;

  return (*this);
}

quat_t quat_t::norm() const {
  quat_t res;
  arm_quaternion_normalize_f32(this->data, res.data, 1);

  return res;
}

quat_t quat_t::conj() const {
  quat_t res;
  arm_quaternion_conjugate_f32(this->data, res.data, 1);

  return res;
}

quat_t operator+(quat_t const &lhs, quat_t const &rhs) {
  quat_t res;
  res.w = lhs.w + rhs.w;
  res.x = lhs.x + rhs.x;
  res.y = lhs.y + rhs.y;
  res.z = lhs.z + rhs.z;

  return res;
}

quat_t operator*(quat_t const &lhs, quat_t const &rhs) {
  quat_t res;
  arm_quaternion_product_f32(lhs.data, rhs.data, res.data, 1);

  return res;
}

quat_t operator*(float32_t lhs, quat_t const &rhs) {
  return rhs * lhs;
}

quat_t operator*(quat_t const &lhs, float32_t rhs) {
  quat_t res;
  res.w = lhs.w * rhs;
  res.x = lhs.x * rhs;
  res.y = lhs.y * rhs;
  res.z = lhs.z * rhs;

  return res;
}

quat_t operator/(quat_t const &lhs, float32_t rhs) {
  quat_t res;
  res.w = lhs.w / rhs;
  res.x = lhs.x / rhs;
  res.y = lhs.y / rhs;
  res.z = lhs.z / rhs;

  return res;
}
