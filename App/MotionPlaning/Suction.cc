#include "MotionPlaning/Suction.h"

/* STM32CubeMX */
#include <main.h>

/* C++ */
#include <cmath>

/* Project */
#include "Config.h"

/* グローバル変数定義 */
extern TIM_HandleTypeDef htim2;

namespace MotionPlaning {

/* 有効化 */
bool Suction::Enable() {
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
  return HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) == HAL_OK;
}
/* 無効化 */
bool Suction::Disable() {
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
  return HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4) == HAL_OK;
}

/* デューティを設定 */
void Suction::SetDuty(float duty) {
  uint32_t dutyNum = static_cast<uint32_t>(static_cast<float>(htim2.Init.Period) * std::abs(duty));
  uint32_t clampDutyNum = std::min(std::max(0ul, dutyNum), htim2.Init.Period);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, clampDutyNum);
}
}  // namespace MotionPlaning
