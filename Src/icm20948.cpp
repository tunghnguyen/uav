#include "icm20948.hpp"

#include "RTE_Components.h"
#include CMSIS_device_header // IWYU pragma: keep

#include "quat.hpp"
#include <cstdint>

ICM20948::ICM20948(I2C_HandleTypeDef &hi2c) : hi2c(hi2c) {
  // Selects IMU user bank 0
  uint8_t reg_bank_sel;
  HAL_I2C_Mem_Read(&hi2c, ICM20948_ADDRESS << 1, ICM20948_PWR_MGMT_1,
                   I2C_MEMADD_SIZE_8BIT, &reg_bank_sel, 1, 1'000);
  CLEAR_BIT(reg_bank_sel, 0x30);
  HAL_I2C_Mem_Write(&hi2c, ICM20948_ADDRESS << 1, ICM20948_REG_BANK_SEL,
                    I2C_MEMADD_SIZE_8BIT, &reg_bank_sel, 1, 1'000);

  // Wakes IMU from sleep mode
  uint8_t pwr_mgmt1;
  HAL_I2C_Mem_Read(&hi2c, ICM20948_ADDRESS << 1, ICM20948_PWR_MGMT_1,
                   I2C_MEMADD_SIZE_8BIT, &pwr_mgmt1, 1, 1'000);
  CLEAR_BIT(reg_bank_sel, 0x40);
  HAL_I2C_Mem_Write(&hi2c, ICM20948_ADDRESS << 1, ICM20948_PWR_MGMT_1,
                    I2C_MEMADD_SIZE_8BIT, &pwr_mgmt1, 1, 1'000);

  uint8_t ctrl = 0x08U;
  HAL_I2C_Mem_Write(&hi2c, AK09916_ADDRESS << 1U, AK09916_USER_CTRL,
                    I2C_MEMADD_SIZE_8BIT, &ctrl, 1, 1'000);
}

quat_t ICM20948::getAccel() const {
  uint8_t accels[6];
  HAL_I2C_Mem_Read(&hi2c, ICM20948_ADDRESS << 1, ICM20948_ACCEL_XOUT_H,
                   I2C_MEMADD_SIZE_8BIT, accels, 6, 1'000);

  quat_t val;
  val.w = 0.0f;
  val.x = (int16_t)((accels[0] << 8) | accels[1]) / accel_sen;
  val.y = (int16_t)((accels[2] << 8) | accels[3]) / accel_sen;
  val.z = (int16_t)((accels[4] << 8) | accels[5]) / accel_sen;

  return val;
}

quat_t ICM20948::getGyro() const {
  uint8_t gyros[6];
  HAL_I2C_Mem_Read(&hi2c, ICM20948_ADDRESS << 1, ICM20948_GYRO_XOUT_H,
                   I2C_MEMADD_SIZE_8BIT, gyros, 6, 1'000);

  quat_t val;
  val.w = 0.0f;
  val.x = (int16_t)((gyros[0] << 8) | gyros[1]) / gyro_sen;
  val.y = (int16_t)((gyros[2] << 8) | gyros[3]) / gyro_sen;
  val.z = (int16_t)((gyros[4] << 8) | gyros[5]) / gyro_sen;

  return val;
}

quat_t ICM20948::getMag() const {
  uint8_t mags[6];
  HAL_I2C_Mem_Read(&hi2c, AK09916_ADDRESS < 1, AK09916_MAG_XOUT_L,
                   I2C_MEMADD_SIZE_8BIT, mags, 6, 1'000);

  quat_t val;
  val.w = 0.0f;
  val.x = (int16_t)(mags[0] | (mags[1] << 8)) * mag_sen;
  val.y = (int16_t)(mags[2] | (mags[3] << 8)) * mag_sen;
  val.z = (int16_t)(mags[4] | (mags[5] << 8)) * mag_sen;

  return val;
}
