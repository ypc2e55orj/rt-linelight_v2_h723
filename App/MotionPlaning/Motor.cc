#include "MotionPlaning/Motor.h"

/* STM32CubeMX */
#include <main.h>

/* C++ */
#include <algorithm>
#include <cmath>

/* グローバル変数定義 */
extern TIM_HandleTypeDef htim1;
static uint32_t channels[] = {TIM_CHANNEL_3, TIM_CHANNEL_4, TIM_CHANNEL_2, TIM_CHANNEL_1};

namespace MotionPlaning {
/* 初期化 */
bool Motor::Initialize() {
  for (int i = 0; i < 4; i++) {
    if (HAL_TIM_PWM_Start(&htim1, channels[i]) != HAL_OK) {
      return false;
    }
  }
  return true;
}

/* 有効化 */
void Motor::Enable() { HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_SET); }
void Motor::Disable() { HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET); }

/* フォールトを取得 */
bool Motor::IsFault() { return HAL_GPIO_ReadPin(DRV_NFAULT_GPIO_Port, DRV_NFAULT_Pin) != GPIO_PIN_RESET; }

/* ブレーキ */
void Motor::Brake() {
  for (int i = 0; i < 4; i++) {
    __HAL_TIM_SET_COMPARE(&htim1, channels[i], 0);
  }
}

/* デューティ設定 */
void Motor::SetDuty(const Duty &duty) {
  for (int i = 0; i < 2; i++) {
    if (std::isfinite(duty[i])) {
      bool ccw = std::signbit(duty[i]);
      uint32_t dutyNum = static_cast<uint32_t>(static_cast<float>(htim1.Init.Period) * std::abs(duty[i]));
      uint32_t clampDutyNum = std::min(std::max(0ul, dutyNum), htim1.Init.Period);
      __HAL_TIM_SET_COMPARE(&htim1, channels[2 * i], ccw ? clampDutyNum : 0);
      __HAL_TIM_SET_COMPARE(&htim1, channels[2 * i + 1], ccw ? 0 : clampDutyNum);
    }
  }
}
}  // namespace MotionPlaning
