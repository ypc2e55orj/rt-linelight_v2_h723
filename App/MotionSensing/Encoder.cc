#include "MotionSensing/Encoder.h"

/* STM32CubeMX */
#include <main.h>

/* C++ */
#include <cmath>
#include <mutex>

/* Project */
#include "Config.h"

/* グローバル変数定義 */
extern TIM_HandleTypeDef htim4; /* 右エンコーダー */
extern TIM_HandleTypeDef htim3; /* 左エンコーダー */
static TIM_HandleTypeDef *encoders[] = {&htim4, &htim3};

namespace MotionSensing {
static uint16_t GetTimCount(TIM_HandleTypeDef *htim, bool invert) {
  return static_cast<uint16_t>(invert ? UINT16_MAX - __HAL_TIM_GET_COUNTER(htim) : __HAL_TIM_GET_COUNTER(htim));
}

/**
 * MARK: Encoder
 */
bool Encoder::Initialize() {
  bool r = HAL_TIM_Encoder_Start(encoders[0], TIM_CHANNEL_ALL) == HAL_OK;
  bool l = HAL_TIM_Encoder_Start(encoders[1], TIM_CHANNEL_ALL) == HAL_OK;
  return r && l;
}

/* リセット */
void Encoder::Reset() {
  std::scoped_lock<Mutex> lock{mtx_};
  for (int i = 0; i < 2; i++) {
    /* 最新値で上書き */
    last_[i] = GetTimCount(encoders[i], i == 0);
  }
  diff_.fill(0);
}

/* 更新 */
void Encoder::Update() {
  std::scoped_lock<Mutex> lock{mtx_};
  for (int i = 0; i < 2; i++) {
    uint16_t curr = GetTimCount(encoders[i], i == 0);
    int32_t delta = CalcWheelDelta(curr, last_[i]);
    diff_[i] = static_cast<float>(delta) * kAnglePerPulse;
    last_[i] = curr;
  }
}

/* カウント値を取得 */
Encoder::Count Encoder::GetCount() {
  std::scoped_lock<Mutex> lock{mtx_};
  return last_;
}

/* 車輪変化角度を取得 [rad] */
Encoder::Diff Encoder::GetDiff() {
  std::scoped_lock<Mutex> lock{mtx_};
  return diff_;
}

/* カウント値から車輪変化角度を算出 */
int32_t Encoder::CalcWheelDelta(uint16_t curr, uint16_t prev) {
  int32_t delta = curr - prev;
  if (std::abs(delta) >= kTimHalfValue) {
    if (prev >= kTimHalfValue) {
      delta += kTimMaxValue;
    } else {
      delta -= kTimMaxValue;
    }
  }
  return delta;
}
}  // namespace MotionSensing
