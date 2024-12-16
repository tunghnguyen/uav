#pragma once

#include "RTE_Components.h"
#include CMSIS_device_header // IWYU pragma: keep

#include "imu.hpp"
#include "quat.hpp"

#include <cstdint>

#define ADDRESS      0x68UL

#define LP_CONFIG    0x05UL
#define PWR_MGMT_1   0x06UL
#define PWR_MGMT_2   0x07UL

#define ACCEL_XOUT_H 0x2DUL
#define ACCEL_XOUT_L 0x2EUL
#define ACCEL_YOUT_H 0x2FUL
#define ACCEL_YOUT_L 0x30UL
#define ACCEL_ZOUT_H 0x31UL
#define ACCEL_ZOUT_L 0x32UL

#define GYRO_XOUT_H  0x33UL
#define GYRO_XOUT_L  0x34UL
#define GYRO_YOUT_H  0x35UL
#define GYRO_YOUT_L  0x36UL
#define GYRO_ZOUT_H  0x37UL
#define GYRO_ZOUT_L  0x38UL

class ICM20948 : public IMU {
private:
  I2C_HandleTypeDef *hi2c;
  float32_t accel_sen = 16'384.0f;
  float32_t gyro_sen = 131.0f;

public:
  ICM20948(I2C_HandleTypeDef *hi2c);

  void setAccelODR(uint32_t rate) override;

  void setGyroODR(uint32_t rate) override;

  void setAccelAAF(uint32_t cutoff) override;

  void setGyroAAF(uint32_t cutoff) override;

  quat_t getAccel() const override;

  quat_t getGyro() const override;
};
