#include "cmsis_os2.h"
#include <cstdint>

#include "RTE_Components.h"
#include CMSIS_device_header // IWYU pragma: keep

#include "Driver_I2C.h"
#include "ICM42605.h"

extern ARM_DRIVER_I2C Driver_I2C1;
#define I2C_A (&Driver_I2C1)

static void app_main(void *argument);

void read(uint8_t addr, uint8_t reg, uint8_t num, uint8_t *val);
void write(uint8_t addr, uint8_t reg, uint8_t num, const uint8_t *val);

int main(void) {
  volatile uint32_t dummy;

  SystemCoreClockUpdate();

  I2C_A->Initialize(NULL);
  I2C_A->PowerControl(ARM_POWER_FULL);
  I2C_A->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_STANDARD);

  uint8_t addr = ICM42605_ADDRESS;
  uint8_t reg  = ICM42605_PWR_MGMT0;
  uint8_t val;

  // Places both the gyroscope and the accelerometer in Low Noise (LN) Mode
  read(addr, reg, 1U, &val);
  write(addr, reg, 1U, &(val |= 0x0FU));

  osKernelInitialize();
  osThreadNew(app_main, NULL, NULL);
  osKernelStart();

  for (;;) {}
}

__NO_RETURN static void app_main(void *argument) {
  (void)argument;

  uint8_t accel[6];
  uint8_t gyro[6];

  for (;;) {
    read(ICM42605_ADDRESS, ICM42605_ACCEL_DATA_X1, 6, accel);
    read(ICM42605_ADDRESS, ICM42605_GYRO_DATA_X1, 6, gyro);

    osDelay(100U);
  }
}

void read(uint8_t addr, uint8_t reg, uint8_t num, uint8_t *val) {
  // Selects the internal register to be read from
  I2C_A->MasterTransmit(addr, &reg, 1U, true);
  while (I2C_A->GetStatus().busy) {}

  // Reads data from the register
  I2C_A->MasterReceive(addr, val, num, false);
  while (I2C_A->GetStatus().busy) {}
}

void write(uint8_t addr, uint8_t reg, uint8_t num, const uint8_t *val) {
  // Transmits the internal register to be written to and then the data
  uint8_t temp[1U + num];
  for (int i = 0; i < num; ++i)
    temp[1U + i] = val[i];

  I2C_A->MasterTransmit(addr, temp, 1U + num, false);
  while (I2C_A->GetStatus().busy) {}
}
