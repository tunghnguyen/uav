#include "I2C.h"

#include "Driver_I2C.h"

#include <cstdint>

extern ARM_DRIVER_I2C Driver_I2C1;
static ARM_DRIVER_I2C &I2C_A = Driver_I2C1;

void I2C::setup(void) {
  I2C_A.Initialize(NULL);
  I2C_A.PowerControl(ARM_POWER_FULL);
  I2C_A.Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_STANDARD);
}

void I2C::read(uint8_t addr, uint8_t reg, uint8_t num, uint8_t *val) {
  // Selects the internal register to be read from
  I2C_A.MasterTransmit(addr, &reg, 1U, true);
  while (I2C_A.GetStatus().busy) {}

  // Reads data from the register
  I2C_A.MasterReceive(addr, val, num, false);
  while (I2C_A.GetStatus().busy) {}
}

void I2C::write(uint8_t addr, uint8_t reg, uint8_t num, const uint8_t *val) {
  // Transmits both the internal register and the data to be written
  uint8_t temp[1U + num];
  temp[0] = reg;
  for (int i = 0; i < num; ++i)
    temp[1U + i] = val[i];

  I2C_A.MasterTransmit(addr, temp, 1U + num, false);
  while (I2C_A.GetStatus().busy) {}
}
