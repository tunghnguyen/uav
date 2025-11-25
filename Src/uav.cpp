#include "main.h"

#include "icm20948.hpp"
// #include "sg90.hpp"

#include "cmsis_os2.h"

#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;
// extern TIM_HandleTypeDef htim2;
// extern TIM_HandleTypeDef htim3;

static osTimerId_t imu_tim;
static osSemaphoreId_t ctrl_semphr;
static osThreadId_t ctrl_handl;

static ICM20948 imu(hi2c1);
// static SG90 servo[4] = {{htim2, TIM_CHANNEL_2},
//                         {htim2, TIM_CHANNEL_3},
//                         {htim3, TIM_CHANNEL_1},
//                         {htim3, TIM_CHANNEL_2}};

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
    HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_5);
    // vec3_t accel = imu.getAccel();
    // printf("%f %f %f\n", accel.x, accel.y, accel.z);
  }
}

int app_main(void) {
  imu.init();
  // for (SG90 &s : servo)
  //   s.init();

  ctrl_semphr = osSemaphoreNew(1, 0, NULL);

  imu_tim = osTimerNew(imuCallback, osTimerPeriodic, NULL, NULL);
  osTimerStart(imu_tim, 100);

  osKernelInitialize();
  osThreadNew(ctrl, NULL, NULL);
  osKernelStart();

  for (;;)
    continue;

  return 0;
}
