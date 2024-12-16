#include "icm20948.hpp"

#include "RTE_Components.h"
#include CMSIS_device_header // IWYU pragma: keep

#include "quat.hpp"

#include <cstdint>

ICM20948::ICM20948(I2C_HandleTypeDef *hi2c) : hi2c(hi2c) {}

void ICM20948::setAccelODR(uint32_t rate) {}

void ICM20948::setGyroODR(uint32_t rate) {}

void ICM20948::setAccelAAF(uint32_t cutoff) {}

void ICM20948::setGyroAAF(uint32_t cutoff) {}

quat_t ICM20948::getAccel() const {
  uint8_t accels[6];
  HAL_I2C_Mem_Read(hi2c, ADDRESS << 1, ACCEL_XOUT_H, 1, accels, 6, 1'000);

  quat_t val;
  val.w = 0.0f;
  val.x = (int16_t)((accels[0] << 8) | accels[1]) / accel_sen;
  val.y = (int16_t)((accels[2] << 8) | accels[3]) / accel_sen;
  val.z = (int16_t)((accels[4] << 8) | accels[5]) / accel_sen;

  return val;
}

quat_t ICM20948::getGyro() const {
  uint8_t gyros[6];
  HAL_I2C_Mem_Read(hi2c, ADDRESS << 1, GYRO_XOUT_H, 1, gyros, 6, 1'000);

  quat_t val;
  val.w = 0.0f;
  val.x = (int16_t)((gyros[0] << 8) | gyros[1]) / gyro_sen;
  val.y = (int16_t)((gyros[2] << 8) | gyros[3]) / gyro_sen;
  val.z = (int16_t)((gyros[4] << 8) | gyros[5]) / gyro_sen;

  return val;
}
