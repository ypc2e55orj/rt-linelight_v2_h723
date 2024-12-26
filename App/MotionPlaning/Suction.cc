#include "MotionPlaning/Suction.h"

/* STM32CubeMX */
#include <main.h>

/* C++ */
#include <cmath>

/* Project */
#include "Config.h"

/* グローバル変数定義 */
extern TIM_HandleTypeDef htim17;

namespace MotionPlaning {

/* 有効化 */
bool Suction::Enable() {
  __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0);
  bool ret = HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1) == HAL_OK;
  bool retEx = HAL_TIMEx_PWMN_Start(&htim17, TIM_CHANNEL_1) == HAL_OK;
  return ret && retEx;
}
/* 無効化 */
bool Suction::Disable() {
  __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, 0);
  bool ret = HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1) == HAL_OK;
  bool retEx = HAL_TIMEx_PWMN_Stop(&htim17, TIM_CHANNEL_1) == HAL_OK;
  return ret && retEx;
}

/* デューティを設定 */
void Suction::SetDuty(float duty) {
  uint32_t compare = static_cast<uint32_t>(static_cast<float>(htim17.Init.Period) * std::abs(duty));
  __HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, compare);
}
}  // namespace MotionPlaning
