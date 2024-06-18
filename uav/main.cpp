#include "I2C.h"
#include "ICM42605.h"

#include "RTE_Components.h"
#include CMSIS_device_header // IWYU pragma: keep
#include "cmsis_os2.h"

#include <cstdint>

static void app_main(void *argument);

int main(void) {
  volatile uint32_t dummy;

  SystemCoreClockUpdate();

  I2C::setup();

  uint8_t addr = ICM42605_ADDRESS;
  uint8_t reg  = ICM42605_PWR_MGMT0;
  uint8_t val;

  // Places both the gyroscope and the accelerometer in Low Noise (LN) Mode
  I2C::read(addr, reg, 1U, &val);
  I2C::write(addr, reg, 1U, &(val |= 0x0FU));

  osKernelInitialize();
  osThreadNew(app_main, NULL, NULL);
  osKernelStart();

  for (;;) {}
}

__NO_RETURN static void app_main(void *argument) {
  (void)argument;

  uint8_t data[12];

  for (;;) {
    I2C::read(ICM42605_ADDRESS, ICM42605_ACCEL_DATA_X1, 12, data);

    osDelay(100U);
  }
}
