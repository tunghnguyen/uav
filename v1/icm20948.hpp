#pragma once

#include "imu.hpp"
#include "quat.hpp"

#include <cstdint>

class ICM20948 : public IMU {
public:
  ICM20948();

  void setAccelODR(uint32_t rate) override;

  void setGyroODR(uint32_t rate) override;

  void setAccelAAF(uint32_t cutoff) override;

  void setGyroAAF(uint32_t cutoff) override;

  quat_t getAccel() const override;

  quat_t getGyro() const override;
};
