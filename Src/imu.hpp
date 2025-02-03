#pragma once

#include "quat.hpp"

class IMU {
public:
  virtual ~IMU() {};

  virtual quat_t getAccel() const = 0;

  virtual quat_t getGyro() const = 0;

  virtual quat_t getMag() const = 0;
};
