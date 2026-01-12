#include "sg90.hpp"

#include "RTE_Components.h"
#include CMSIS_device_header // IWYU pragma: keep

#include "arm_math_types.h"

SG90::SG90(TIM_HandleTypeDef &htim, uint32_t chan) : htim(htim), chan(chan) {}

void SG90::init() {
  ccr_min = (__HAL_TIM_GET_AUTORELOAD(&htim) + 1) / 20;
  ccr_max = (__HAL_TIM_GET_AUTORELOAD(&htim) + 1) / 10;

  HAL_TIM_PWM_Start(&htim, chan);
}

void SG90::setPos(float32_t pos) {
  uint32_t tmp = (ccr_min + (ccr_max - ccr_min) * pos);
  __HAL_TIM_SET_COMPARE(&htim, chan, tmp - 1);
}
