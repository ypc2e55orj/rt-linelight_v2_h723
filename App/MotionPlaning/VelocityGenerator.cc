#include "MotionPlaning/VelocityGenerator.h"

/* C++ */
#include <cmath>

/* 計算 */
void SlopeVelocityGenerator::Generate(const Profile &profile) {
  profile_ = profile;
  if (std::abs(profile_.distance) > 0.0f) {
    /* 移動距離がが定まっている場合 */
    t0_ = (profile_.maxVelocity - profile_.startVelocity) / profile_.acceleration;
    float x0 = profile_.startVelocity * t0_ + (profile_.acceleration * powf(t0_, 2)) / 2.0f;
    t2_ = (profile_.endVelocity - profile_.maxVelocity) / profile_.deceleration;
    float x2 = profile_.maxVelocity * t2_ + (profile_.deceleration * powf(t2_, 2)) / 2.0f;
    float x1 = profile_.distance - x0 - x2;
    t1_ = x1 / profile_.maxVelocity;
  }
}
/* 時刻tにおける速度を取得 */
float SlopeVelocityGenerator::GetVelocity(uint32_t ms) {
  float t = static_cast<float>(ms) / 1000.0f;
  if (t < t0_) {
    return profile_.startVelocity + profile_.acceleration * t;
  } else if (t < (t0_ + t1_)) {
    return profile_.maxVelocity;
  } else if (t < (t0_ + t1_ + t2_)) {
    return profile_.maxVelocity + profile_.deceleration * (t - (t0_ + t1_));
  }
  return 0.0f;
}

/* 加速時間を取得 */
uint32_t SlopeVelocityGenerator::GetAccelerationTime() { return static_cast<uint32_t>(t0_ * 1000.0f); }
/* 加速距離を取得 */
float SlopeVelocityGenerator::GetAccelerationDistance() { return x0_; }
/* 等速時間を取得 */
uint32_t SlopeVelocityGenerator::GetConstantSpeedTime() { return static_cast<uint32_t>(t1_ * 1000.0f); }
/* 等速距離を取得 */
float SlopeVelocityGenerator::GetConstantSpeedDistance() { return x1_; }
/* 減速時間を取得 */
uint32_t SlopeVelocityGenerator::GetDecelerationTime() { return static_cast<uint32_t>(t2_ * 1000.0f); }
/* 減速距離を取得 */
float SlopeVelocityGenerator::GetDecelerationDistance() { return x2_; }
/* 走行時間を取得 */
uint32_t SlopeVelocityGenerator::GetTotalTime() { return static_cast<uint32_t>((t0_ + t1_ + t2_) * 1000.0f); }
/* 走行距離を取得 */
float SlopeVelocityGenerator::GetTotalDistance() { return (x0_ + x1_ + x2_); }
