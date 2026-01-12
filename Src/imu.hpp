#pragma once

#include "quat.hpp"

class IMU {
public:
  virtual ~IMU() {};

  virtual vec3_t getAccel() const = 0;

  virtual vec3_t getGyro() const = 0;

  virtual vec3_t getMag() const = 0;
};
