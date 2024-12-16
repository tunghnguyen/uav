#pragma once

#include "quat.hpp"

#include <cstdint>

class IMU {
public:
  virtual ~IMU() {};

  virtual void setAccelODR(uint32_t rate) = 0;

  virtual void setGyroODR(uint32_t rate) = 0;

  virtual void setAccelAAF(uint32_t cutoff) = 0;

  virtual void setGyroAAF(uint32_t cutoff) = 0;

  virtual quat_t getAccel() const = 0;

  virtual quat_t getGyro() const = 0;
};
