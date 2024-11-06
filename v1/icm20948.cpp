#include "icm20948.hpp"

#include "RTE_Components.h"
#include CMSIS_device_header // IWYU pragma: keep

#include "quat.hpp"
#include "stm32f7xx_hal_i2c.h"

#include <cstdint>

quat_t fuse() {
  // TODO: Pass these in
  static quat_t orient{.w = 0.0f, .x = 0.0f, .y = 0.0f, .z = 1.0f};
  quat_t accel;
  quat_t gyro;

  float32_t dt = 0.01f;

  float32_t alpha = 0.05f;

  // Normalizes acceleration
  accel.normalize();

  // Integrates angular velocity
  orient += 0.5f * orient * gyro * dt;
  orient.normalize();

  // Fuses the sensors
  orient = accel * alpha + orient * (1.0f - alpha);
  orient.normalize();

  return orient;
}

ICM20948::ICM20948(I2C_HandleTypeDef *hi2c) : hi2c(hi2c) {
  uint8_t pwr_mgmt;
  HAL_I2C_Mem_Read(hi2c, ADDRESS << 1, PWR_MGMT_1, 1, &pwr_mgmt, 1, 1000);

  // Wakes the IMU from sleep mode
  pwr_mgmt &= ~0x40;
  HAL_I2C_Mem_Write(hi2c, ADDRESS << 1, PWR_MGMT_1, 1, &pwr_mgmt, 1, 1000);
}

void ICM20948::setAccelODR(uint32_t rate) {}

void ICM20948::setGyroODR(uint32_t rate) {}

void ICM20948::setAccelAAF(uint32_t cutoff) {}

void ICM20948::setGyroAAF(uint32_t cutoff) {}

quat_t ICM20948::getAccel() const {
  uint8_t accel_out[6];
  HAL_I2C_Mem_Read(hi2c, ADDRESS << 1, ACCEL_XOUT_H, 1, accel_out, 6, 1000);

  quat_t val;
  val.w = 0.0f;
  val.x = ((uint16_t)accel_out[0] << 8) | accel_out[1];
  val.y = ((uint16_t)accel_out[2] << 8) | accel_out[3];
  val.z = ((uint16_t)accel_out[4] << 8) | accel_out[5];

  return val / accel_sen;
}

quat_t ICM20948::getGyro() const {
  uint8_t gyro_out[6];
  HAL_I2C_Mem_Read(hi2c, ADDRESS << 1, GYRO_XOUT_H, 1, gyro_out, 6, 1000);

  quat_t val;
  val.w = 0.0f;
  val.x = ((uint16_t)gyro_out[0] << 8) | gyro_out[1];
  val.y = ((uint16_t)gyro_out[2] << 8) | gyro_out[3];
  val.z = ((uint16_t)gyro_out[4] << 8) | gyro_out[5];

  return val / gyro_sen;
}
