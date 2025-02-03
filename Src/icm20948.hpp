#pragma once

#include "RTE_Components.h"
#include CMSIS_device_header // IWYU pragma: keep

#include "imu.hpp"
#include "quat.hpp"

#define ICM20948_ADDRESS      0x68UL
#define AK09916_ADDRESS       0x0CUL

#define ICM20948_REG_BANK_SEL 0x7FUL

// User Bank 0
#define WHO_AM_I              0x00UL
#define ICM20948_LP_CONFIG    0x05UL
#define ICM20948_PWR_MGMT_1   0x06UL
#define ICM20948_PWR_MGMT_2   0x07UL

#define ICM20948_ACCEL_XOUT_H 0x2DUL
#define ICM20948_ACCEL_XOUT_L 0x2EUL
#define ICM20948_ACCEL_YOUT_H 0x2FUL
#define ICM20948_ACCEL_YOUT_L 0x30UL
#define ICM20948_ACCEL_ZOUT_H 0x31UL
#define ICM20948_ACCEL_ZOUT_L 0x32UL

#define ICM20948_GYRO_XOUT_H  0x33UL
#define ICM20948_GYRO_XOUT_L  0x34UL
#define ICM20948_GYRO_YOUT_H  0x35UL
#define ICM20948_GYRO_YOUT_L  0x36UL
#define ICM20948_GYRO_ZOUT_H  0x37UL
#define ICM20948_GYRO_ZOUT_L  0x38UL

// AK09916 Bank
#define AK09916_USER_CTRL     0x31UL

#define AK09916_MAG_XOUT_L    0x11UL
#define AK09916_MAG_XOUT_H    0x12UL
#define AK09916_MAG_YOUT_L    0x13UL
#define AK09916_MAG_YOUT_H    0x14UL
#define AK09916_MAG_ZOUT_L    0x15UL
#define AK09916_MAG_ZOUT_H    0x16UL

class ICM20948 : public IMU {
private:
  I2C_HandleTypeDef &hi2c;

  const float32_t accel_sen = 16'384.0f; // (LSB/dsp)
  const float32_t gyro_sen = 131.0f;     // (LSB/g)
  const float32_t mag_sen = 0.15f;       // (uT/LSB)

public:
  ICM20948(I2C_HandleTypeDef &hi2c);

  quat_t getAccel() const override;

  quat_t getGyro() const override;

  quat_t getMag() const override;
};
