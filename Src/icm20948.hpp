#pragma once

#include "RTE_Components.h"
#include CMSIS_device_header // IWYU pragma: keep

#include "buffer.hpp"
#include "imu.hpp"
#include "quat.hpp"

#include "arm_math_types.h"
#include "dsp/fast_math_functions.h"

#define ICM20948_ADDRESS            0x68UL
#define ICM20948_WHO_AM_I_DEFAULT   0xEAUL

// User Bank Select
#define ICM20948_REG_BANK_SEL       0x7FUL

// User Bank 0
#define ICM20948_WHO_AM_I           0x00UL
#define ICM20948_USER_CTRL          0x03UL
#define ICM20948_LP_CONFIG          0x05UL
#define ICM20948_PWR_MGMT_1         0x06UL
#define ICM20948_PWR_MGMT_2         0x07UL

#define ICM20948_INT_PIN_CFG        0x0FUL
#define ICM20948_INT_ENABLE         0x10UL
#define ICM20948_INT_ENABLE_1       0x11UL
#define ICM20948_INT_ENABLE_2       0x12UL
#define ICM20948_INT_ENABLE_3       0x13UL
#define ICM20948_I2C_MST_STATUS     0x17UL
#define ICM20948_INT_STATUS         0x19UL
#define ICM20948_INT_STATUS_1       0x1AUL
#define ICM20948_INT_STATUS_2       0x1BUL
#define ICM20948_INT_STATUS_3       0x1CUL
#define ICM20948_DELAY_TIMEH        0x28UL
#define ICM20948_DELAY_TIMEL        0x29UL

#define ICM20948_ACCEL_XOUT_H       0x2DUL
#define ICM20948_ACCEL_XOUT_L       0x2EUL
#define ICM20948_ACCEL_YOUT_H       0x2FUL
#define ICM20948_ACCEL_YOUT_L       0x30UL
#define ICM20948_ACCEL_ZOUT_H       0x31UL
#define ICM20948_ACCEL_ZOUT_L       0x32UL
#define ICM20948_GYRO_XOUT_H        0x33UL
#define ICM20948_GYRO_XOUT_L        0x34UL
#define ICM20948_GYRO_YOUT_H        0x35UL
#define ICM20948_GYRO_YOUT_L        0x36UL
#define ICM20948_GYRO_ZOUT_H        0x37UL
#define ICM20948_GYRO_ZOUT_L        0x38UL
#define ICM20948_TEMP_OUT_H         0x39UL
#define ICM20948_TEMP_OUT_L         0x3AUL

#define ICM20948_FIFO_EN_1          0x66UL
#define ICM20948_FIFO_EN_2          0x67UL
#define ICM20948_FIFO_RST           0x68UL
#define ICM20948_FIFO_MODE          0x69UL
#define ICM20948_FIFO_COUNTH        0x70UL
#define ICM20948_FIFO_COUNTL        0x71UL
#define ICM20948_FIFO_R_W           0x72UL
#define ICM20948_DATA_RDY_STATUS    0x74UL
#define ICM20948_FIFO_CFG           0x76UL

// User Bank 2
#define ICM20948_GYRO_SMPLRT_DIV    0x00UL
#define ICM20948_GYRO_CONFIG_1      0x01UL
#define ICM20948_GYRO_CONFIG_2      0x02UL
#define ICM20948_ODR_ALIGN_EN       0x09UL
#define ICM20948_ACCEL_SMPLRT_DIV_1 0x10UL
#define ICM20948_ACCEL_SMPLRT_DIV_2 0x11UL
#define ICM20948_ACCEL_INTEL_CTRL   0x12UL
#define ICM20948_ACCEL_WOM_THR      0x13UL
#define ICM20948_ACCEL_CONFIG       0x14UL
#define ICM20948_ACCEL_CONFIG_2     0x15UL
#define ICM20948_FSYNC_CONFIG       0x52UL
#define ICM20948_TEMP_CONFIG        0x53UL
#define ICM20948_MOD_CTRL_USR       0x54UL

// User Bank 3
#define ICM20948_I2C_MST_ODR_CONFIG 0x00UL
#define ICM20948_I2C_MST_CTRL       0x01UL
#define ICM20948_I2C_MST_DELAY_CTRL 0x02UL

#define ICM20948_I2C_SLV0_ADDR      0x03UL
#define ICM20948_I2C_SLV0_REG       0x04UL
#define ICM20948_I2C_SLV0_CTRL      0x05UL
#define ICM20948_I2C_SLV0_DO        0x06UL
#define ICM20948_I2C_SLV1_ADDR      0x07UL
#define ICM20948_I2C_SLV1_REG       0x08UL
#define ICM20948_I2C_SLV1_CTRL      0x09UL
#define ICM20948_I2C_SLV1_DO        0x0AUL
#define ICM20948_I2C_SLV2_ADDR      0x0BUL
#define ICM20948_I2C_SLV2_REG       0x0CUL
#define ICM20948_I2C_SLV2_CTRL      0x0DUL
#define ICM20948_I2C_SLV2_DO        0x0EUL
#define ICM20948_I2C_SLV3_ADDR      0x0FUL
#define ICM20948_I2C_SLV3_REG       0x10UL
#define ICM20948_I2C_SLV3_CTRL      0x11UL
#define ICM20948_I2C_SLV3_DO        0x12UL
#define ICM20948_I2C_SLV4_ADDR      0x13UL
#define ICM20948_I2C_SLV4_REG       0x14UL
#define ICM20948_I2C_SLV4_CTRL      0x15UL
#define ICM20948_I2C_SLV4_DO        0x16UL
#define ICM20948_I2C_SLV4_DI        0x17UL

class ICM20948 : public IMU {
private:
  I2C_HandleTypeDef &hi2c;

  const float32_t accel_sen = 16'384.0 / (PI / 180.0); // [ LSB/(rad/s) ]
  const float32_t gyro_sen = 131.0 / 9.80665;          // [ LSB/(m/s^2) ]
  const float32_t mag_sen = 0.15;                      // [   uT/LSB    ]
  const uint8_t odr = 125;                             // [     Hz      ]

  uint8_t sensors_buf[12];

  Buffer<vec3_t, 4> accel;
  Buffer<vec3_t, 4> gyro;
  Buffer<vec3_t, 4> mag;

  void modReg(uint16_t reg, uint8_t clr_msk, uint8_t set_msk);

  void setReg(uint16_t reg, uint8_t val);

  uint8_t getReg(uint16_t reg);

  void updateBuf();

public:
  ICM20948(I2C_HandleTypeDef &hi2c);

  void init();

  void startSensorsIT();

  void complSensorsIT();

  void startSensorsPoll();

  vec3_t getAccel() const override;

  vec3_t getGyro() const override;

  vec3_t getMag() const override;
};
