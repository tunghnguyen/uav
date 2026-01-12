#pragma once

#include "RTE_Components.h"
#include CMSIS_device_header // IWYU pragma: keep

#include "arm_math_types.h"

class SG90 {
private:
  TIM_HandleTypeDef &htim;
  uint32_t chan;

  uint32_t ccr_min;
  uint32_t ccr_max;

public:
  SG90(TIM_HandleTypeDef &htim, uint32_t chan);

  void init();

  void setPos(float32_t pos);
};
