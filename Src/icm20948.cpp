#include "icm20948.hpp"

#include "RTE_Components.h"
#include CMSIS_device_header // IWYU pragma: keep

#include "buffer.cpp"
#include "quat.hpp"

#include <cstdint>

ICM20948::ICM20948(I2C_HandleTypeDef &hi2c) : hi2c(hi2c) {}

void ICM20948::init() {
  // Wakes from sleep mode
  modReg(ICM20948_REG_BANK_SEL, 0x00, 0x00);
  modReg(ICM20948_PWR_MGMT_1, 0x40, 0x00);
}

void ICM20948::startSensorsIT() {
  HAL_I2C_Mem_Read_IT(&hi2c, ICM20948_ADDRESS << 1, ICM20948_ACCEL_XOUT_H,
                      I2C_MEMADD_SIZE_8BIT, sensors_buf, 12);
}

void ICM20948::complSensorsIT() {
  updateBuf();
}

void ICM20948::startSensorsPoll() {
  HAL_I2C_Mem_Read(&hi2c, ICM20948_ADDRESS << 1, ICM20948_ACCEL_XOUT_H,
                   I2C_MEMADD_SIZE_8BIT, sensors_buf, 12, 100);
  updateBuf();
}

vec3_t ICM20948::getAccel() const {
  return accel.read();
}

vec3_t ICM20948::getGyro() const {
  return gyro.read();
}

vec3_t ICM20948::getMag() const {
  return mag.read();
}

void ICM20948::modReg(uint16_t reg, uint8_t clr_msk, uint8_t set_msk) {
  uint8_t tmp;
  HAL_I2C_Mem_Read(&hi2c, ICM20948_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT,
                   &tmp, 1, 1'000);
  MODIFY_REG(tmp, clr_msk, set_msk);
  HAL_I2C_Mem_Write(&hi2c, ICM20948_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT,
                    &tmp, 1, 1'000);
}

void ICM20948::setReg(uint16_t reg, uint8_t val) {
  uint8_t tmp = val;
  HAL_I2C_Mem_Write(&hi2c, ICM20948_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT,
                    &tmp, 1, 1'000);
}

uint8_t ICM20948::getReg(uint16_t reg) {
  uint8_t tmp;
  HAL_I2C_Mem_Read(&hi2c, ICM20948_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT,
                   &tmp, 1, 1'000);
  return tmp;
}

void ICM20948::updateBuf() {
  vec3_t accel_tmp;
  accel_tmp.x = (int16_t)((sensors_buf[0] << 8) | sensors_buf[1]) / accel_sen;
  accel_tmp.y = (int16_t)((sensors_buf[2] << 8) | sensors_buf[3]) / accel_sen;
  accel_tmp.z = (int16_t)((sensors_buf[4] << 8) | sensors_buf[5]) / accel_sen;

  accel.write(accel_tmp);

  vec3_t gyro_tmp;
  gyro_tmp.x = (int16_t)((sensors_buf[6] << 8) | sensors_buf[7]) / gyro_sen;
  gyro_tmp.y = (int16_t)((sensors_buf[8] << 8) | sensors_buf[9]) / gyro_sen;
  gyro_tmp.z = (int16_t)((sensors_buf[10] << 8) | sensors_buf[11]) / gyro_sen;

  gyro.write(gyro_tmp);
}
