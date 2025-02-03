#include "main.h"

#include "icm20948.hpp"
#include "quat.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "stdio.h"
#include "task.h"

extern I2C_HandleTypeDef hi2c1;

int app_main(void) {
  ICM20948 icm20948(hi2c1);
  IMU *imu = &icm20948;

  quat_t accel;
  quat_t gyro;

  for (;;) {
    accel = imu->getAccel();
    gyro = imu->getGyro();
    printf("\n%.2f, %.2f, %.2f", accel.x, accel.y, accel.z);
    printf("\n%.2f, %.2f, %.2f", gyro.x, gyro.y, gyro.z);
    HAL_Delay(500);
  }

  return 0;
}
