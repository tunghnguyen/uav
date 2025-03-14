#include "main.h"

#include "icm20948.hpp"
// #include "quat.hpp"

#include "cmsis_os2.h"

#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;

static osTimerId_t imu_tim;
static osSemaphoreId_t ctrl_semphr;
static osThreadId_t ctrl_handl;

static ICM20948 imu(hi2c1);

void imuCallback(void *arg) {
  imu.startSensorsIT();
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c == &hi2c1) {
    imu.complSensorsIT();
    osSemaphoreRelease(ctrl_semphr);
  }
}

void ctrl(void *arg) {
  for (;;) {
    osSemaphoreAcquire(ctrl_semphr, osWaitForever);
    // TODO: Use sensors
  }
}

int app_main(void) {
  imu.init();

  ctrl_semphr = osSemaphoreNew(1, 0, NULL);

  imu_tim = osTimerNew(imuCallback, osTimerPeriodic, NULL, NULL);
  osTimerStart(imu_tim, 1'000);

  osKernelInitialize();
  osThreadNew(ctrl, NULL, NULL);
  osKernelStart();

  for (;;);

  return 0;
}
