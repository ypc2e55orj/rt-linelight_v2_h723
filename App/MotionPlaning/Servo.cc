#include "MotionPlaning/Servo.h"

/* プロジェクト */
#include "Config.h"
#include "Data/Pid.h"

/* C++ */
#include <algorithm>
#include <cmath>
#include <mutex>

namespace MotionPlaning {
/* ゲインを設定 */
void ServoImpl::SetGain(const Pid::Gain &linear, const Pid::Gain &angular) {
  std::scoped_lock<Mutex> lock(mtx_);
  pidLinear_.Reset(linear);
  pidAngular_.Reset(angular);
}

/* 目標値を設定 */
void ServoImpl::SetTarget(float linear, float angular) {
  std::scoped_lock<Mutex> lock(mtx_);
  targetLinear_ = linear;
  targetAngular_ = angular;
}

/* リセット */
void ServoImpl::Reset() {
  std::scoped_lock<Mutex> lock(mtx_);
  hasError_ = false;
  pidLinear_.Reset();
  pidAngular_.Reset();
}

/* 更新 */
void ServoImpl::Update(float batteryVoltage, /* バッテリー電圧 [V] */
                   float measureLinear,  /* 速度 [m/s] */
                   float measureAccel,   /* 加速度 [m/ss] */
                   float measureAngular, /* 角速度 [rad/s] */
                   float measureAlpha    /* 角加速度 [rad/ss] */
) {
  static constexpr float K = kTorqueConstant;                                              /* トルク定数 [N*m/A] */
  static constexpr float N = kGearRatio;                                                   /* ギア比 */
  static constexpr float W = kTreadWidth;                                                  /* トレッド幅 [m] */
  static constexpr float R = kWheelRadius;                                                 /* 車輪半径 [m] */
  static constexpr float M = kMachineWeight * kFeedForwardLinearGain;                      /* 車体質量 [kg] */
  static constexpr float J = kMachineWeight * (W / 2) * (W / 2) * kFeedForwardAngularGain; /* イナーシャ [kg*m^2] */
  static constexpr float kRadPerSecToRpm = (60.0f * kGearRatio) / (2.0f * static_cast<float>(M_PI));

  std::scoped_lock<Mutex> lock(mtx_);

  /* フィードフォワード制御 */
  /* 要求モーター回転数を計算 */
  float wheelOmegaRight = kRadPerSecToRpm * (targetLinear_ / R + (W * targetAngular_) / (2.0f * R));
  float wheelOmegaLeft = kRadPerSecToRpm * (targetLinear_ / R - (W * targetAngular_) / (2.0f * R));
  /* 要求電流を計算 */
  float currentRight = ((R / 2.0f) * (M * measureAccel) + (R / W) * (J * measureAlpha)) / (K * N);
  float currentLeft = ((R / 2.0f) * (M * measureAccel) - (R / W) * (J * measureAlpha)) / (K * N);
  /* FF項を電圧に変換 */
  feedforward_ = {
      currentRight * kMotorResistance + kMotorBackEmf * wheelOmegaRight,
      currentLeft * kMotorResistance + kMotorBackEmf * wheelOmegaLeft,
  };

  /* フィードバック制御 */
  feedback_ = {
      pidLinear_.Update(targetLinear_, measureLinear, 1.0f),
      pidAngular_.Update(targetAngular_, measureAngular, 1.0f),
  };

  /* 電圧に換算 */
  voltage_ = {
      feedforward_[0] + feedback_[0] + feedback_[1],
      feedforward_[1] + feedback_[0] - feedback_[1],
  };

  /* NaN・Infを弾く */
  if (!std::isfinite(voltage_[0]) || !std::isfinite(voltage_[1])) {
    hasError_ = true;
    return;
  }

  /* 上限電圧で丸める */
  for (auto &voltage : voltage_) {
    voltage = std::copysign(std::min(std::abs(voltage), kMotorLimitVoltage), voltage);
  }

  duty_ = {
      voltage_[0] / batteryVoltage,
      voltage_[1] / batteryVoltage,
  };
}

/* 設定モーター電圧を取得 */
ServoImpl::ControlAmount ServoImpl::GetMotorVoltage() {
  std::scoped_lock<Mutex> lock(mtx_);
  return voltage_;
}

/* デューティ比を取得 */
ServoImpl::ControlAmount ServoImpl::GetMotorDuty() {
  std::scoped_lock<Mutex> lock(mtx_);
  return duty_;
}

/* フィードフォワード制御量を取得 */
ServoImpl::ControlAmount ServoImpl::GetFeedForwardAmount() {
  std::scoped_lock<Mutex> lock(mtx_);
  return feedforward_;
}
/* フィードバック制御量を取得 */
ServoImpl::ControlAmount ServoImpl::GetFeedBackAmount() {
  std::scoped_lock<Mutex> lock(mtx_);
  return feedback_;
}

/* エラーが発生したか */
bool ServoImpl::HasError() { return hasError_; }
}  // namespace MotionPlaning
