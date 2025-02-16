#ifndef MOTIONPLANING_SERVO_H_
#define MOTIONPLANING_SERVO_H_

/* C++ */
#include <array>

/* Project */
#include "Data/Pid.h"
#include "Wrapper/Mutex.h"

namespace MotionPlaning {
class ServoImpl {
 public:
  using ControlAmount = std::array<float, 2>;

  /* ゲインを設定 */
  void SetGain(const Pid::Gain &linear, const Pid::Gain &angular);

  /* 目標値を設定 */
  void SetTarget(float linear, float angular);

  /* リセット */
  void Reset();

  /* 更新 */
  void Update(float batteryVoltage, /* バッテリー電圧 [V] */
              float measureLinear,  /* 速度 [m/s] */
              float measureAngular  /* 角速度 [rad/s] */
  );

  /* 設定モーター電圧を取得 */
  ControlAmount GetMotorVoltage();

  /* デューティ比を取得 */
  ControlAmount GetMotorDuty();

  /* 制御量を取得 */
  ControlAmount GetFeedForwardOmega();
  ControlAmount GetFeedForwardCurrent();
  ControlAmount GetFeedForwardAmount();
  ControlAmount GetFeedBackAmount();

  /* エラーが発生したかを取得 */
  bool HasError();

 private:
  mutable Mutex mtx_;

  Pid pidLinear_;
  Pid pidAngular_;

  float targetLinear_;
  float targetAngular_;

  ControlAmount feedforwardWheelOmega_{};

  ControlAmount feedforward_{};
  ControlAmount feedback_{};
  ControlAmount voltage_{};
  ControlAmount duty_{};

  bool hasError_;
};
}  // namespace MotionPlaning
#endif  // MOTIONPLANING_SERVO_H_
