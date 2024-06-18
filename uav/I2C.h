#pragma once

#include <cstdint>

namespace I2C {

/**
 * @brief
 *
 */
void setup(void);

/**
 * @brief
 *
 * @param addr
 * @param reg
 * @param num
 * @param val
 */
void read(uint8_t addr, uint8_t reg, uint8_t num, uint8_t *val);

/**
 * @brief
 *
 * @param addr
 * @param reg
 * @param num
 * @param val
 */
void write(uint8_t addr, uint8_t reg, uint8_t num, const uint8_t *val);

} // namespace I2C
