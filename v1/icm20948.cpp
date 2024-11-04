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

ICM20948::ICM20948() {}

void ICM20948::setAccelODR(uint32_t rate) {}

void ICM20948::setAccelAAF(uint32_t cutoff) {}

void ICM20948::setGyroODR(uint32_t rate) {}

void ICM20948::setGyroAAF(uint32_t cutoff) {}

quat_t ICM20948::getAccel() const {}

quat_t ICM20948::getGyro() const {}
