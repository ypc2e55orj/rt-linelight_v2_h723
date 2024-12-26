#include "MotionPlaning/Motor.h"

/* STM32CubeMX */
#include <main.h>

/* C++ */
#include <algorithm>
#include <cmath>

/* グローバル変数定義 */
extern TIM_HandleTypeDef htim4; /* 右モーター */
extern TIM_HandleTypeDef htim2; /* 左モーター */

static TIM_HandleTypeDef *tim[] = {&htim4, &htim2};
static uint32_t ch1[] = {TIM_CHANNEL_2, TIM_CHANNEL_1};
static uint32_t ch2[] = {TIM_CHANNEL_3, TIM_CHANNEL_3};
static GPIO_TypeDef *enablePort[] = {DRIVER_RIGHT_EN_GPIO_Port, DRIVER_LEFT_EN_GPIO_Port};
static uint16_t enablePin[] = {DRIVER_RIGHT_EN_Pin, DRIVER_LEFT_EN_Pin};
static GPIO_TypeDef *faultPort[] = {DRIVER_RIGHT_NFAULT_GPIO_Port, DRIVER_LEFT_NFAULT_GPIO_Port};
static uint16_t faultPin[] = {DRIVER_RIGHT_NFAULT_Pin, DRIVER_LEFT_NFAULT_Pin};

namespace MotionPlaning {
/* 初期化 */
bool Motor::Initialize() {
  for (int i = 0; i < 2; i++) {
    if (HAL_TIM_PWM_Start(tim[i], ch1[i]) != HAL_OK || HAL_TIM_PWM_Start(tim[i], ch2[i]) != HAL_OK) {
      return false;
    }
  }
  return true;
}

/* 有効化 */
void Motor::Enable() {
  for (int i = 0; i < 2; i++) {
    HAL_GPIO_WritePin(enablePort[i], enablePin[i], GPIO_PIN_SET);
  }
}
void Motor::Disable() {
  for (int i = 0; i < 2; i++) {
    HAL_GPIO_WritePin(enablePort[i], enablePin[i], GPIO_PIN_RESET);
  }
}

/* フォールトを取得 */
bool Motor::IsFault() {
  for (int i = 0; i < 2; i++) {
    if (HAL_GPIO_ReadPin(faultPort[i], faultPin[i]) != GPIO_PIN_RESET) {
      return false;
    }
  }
  return true;
}

/* フリーに設定 */
void Motor::SetFree() {
  for (int i = 0; i < 2; i++) {
    __HAL_TIM_SET_COMPARE(tim[i], ch1[i], 0);
    __HAL_TIM_SET_COMPARE(tim[i], ch2[i], 0);
  }
}

/* デューティ設定 */
void Motor::SetDuty(const Duty &duty) {
  for (int i = 0; i < 2; i++) {
    if (std::isfinite(duty[i])) {
      bool ccw = std::signbit(duty[i]);
      uint32_t dutyNum = static_cast<uint32_t>(static_cast<float>(tim[i]->Init.Period) * std::abs(duty[i]));
      uint32_t clampDutyNum = std::min(std::max(0ul, dutyNum), tim[i]->Init.Period);
      __HAL_TIM_SET_COMPARE(tim[i], ch1[i], ccw ? clampDutyNum : 0);
      __HAL_TIM_SET_COMPARE(tim[i], ch2[i], ccw ? 0 : clampDutyNum);
    }
  }
}
}  // namespace MotionPlaning
