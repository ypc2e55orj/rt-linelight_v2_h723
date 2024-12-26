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
void Servo::SetGain(const Pid::Gain &linear, const Pid::Gain &angular) {
  std::scoped_lock<Mutex> lock(mtx_);
  pidLinear_.Reset(linear);
  pidAngular_.Reset(angular);
}

/* 目標値を設定 */
void Servo::SetTarget(float linear, float angular) {
  std::scoped_lock<Mutex> lock(mtx_);
  targetLinear_ = linear;
  targetAngular_ = angular;
}

/* リセット */
void Servo::Reset() {
  std::scoped_lock<Mutex> lock(mtx_);
  pidLinear_.Reset();
  pidAngular_.Reset();
}

/* 更新 */
void Servo::Update(float batteryVoltage, /* バッテリー電圧 [V] */
                   float measureLinear,  /* 速度 [m/s] */
                   float measureAccel,   /* 加速度 [m/ss] */
                   float measureAngular, /* 角速度 [rad/s] */
                   float measureAlpha    /* 角加速度 [rad/ss] */
) {
  static constexpr float K = kTorqueConstant;                                 /* トルク定数 [N*m/A] */
  static constexpr float N = kGearRatio;                                      /* ギア比 */
  static constexpr float W = kTreadWidth;                                     /* トレッド幅 [m] */
  static constexpr float R = kWheelRadius;                                    /* 車輪半径 [m] */
  static constexpr float M = kMachineWeight * kFeedForwardLinearGain;         /* 車体質量 [kg] */
  static constexpr float J = M * (W / 2) * (W / 2) * kFeedForwardAngularGain; /* イナーシャ [kg*m^2] */
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
  float ffVoltageRight = currentRight * kMotorResistance + kMotorBackEmf * wheelOmegaRight;
  float ffVoltageLeft = currentLeft * kMotorResistance + kMotorBackEmf * wheelOmegaLeft;

  /* フィードバック制御 */
  float errorLinear = pidLinear_.Update(targetLinear_, measureLinear, 1.0f);
  float errorAngular = pidAngular_.Update(targetAngular_, measureAngular, 1.0f);

  /* 電圧に換算 */
  float voltageRight = ffVoltageRight + errorLinear + errorAngular;
  float voltageLeft = ffVoltageLeft + errorLinear - errorAngular;
  voltage_ = {voltageRight, voltageLeft};
  if (std::abs(voltageRight) > kMotorLimitVoltage || std::abs(voltageLeft) > kMotorLimitVoltage) {
    float factor = kMotorLimitVoltage / std::max(std::abs(voltageRight), std::abs(voltageLeft));
    voltageRight *= factor;
    voltageLeft *= factor;
    isLimited_ = true;
  } else {
    isLimited_ = false;
  }
  if (batteryVoltage > kBatteryVoltageLimitMin && std::isfinite(voltageRight) && std::isfinite(voltageLeft)) {
    duty_[0] = voltageRight / batteryVoltage;
    duty_[1] = voltageLeft / batteryVoltage;
  } else {
    duty_.fill(0.0f);
  }
}

/* 設定モーター電圧を取得 */
Servo::Voltage Servo::GetMotorVoltage() {
  std::scoped_lock<Mutex> lock(mtx_);
  return voltage_;
}

/* デューティ比を取得 */
Servo::Duty Servo::GetMotorDuty() {
  std::scoped_lock<Mutex> lock(mtx_);
  return duty_;
}

/* 電圧が丸められたかを取得 */
bool Servo::IsLimited() {
  std::scoped_lock<Mutex> lock(mtx_);
  return isLimited_;
}
}  // namespace MotionPlaning
