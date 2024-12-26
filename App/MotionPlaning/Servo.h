#ifndef MOTIONPLANING_SERVO_H_
#define MOTIONPLANING_SERVO_H_

/* C++ */
#include <array>

/* Project */
#include "Data/Pid.h"
#include "Wrapper/Mutex.h"

namespace MotionPlaning {
class Servo {
 public:
  using Voltage = std::array<float, 2>;
  using Duty = std::array<float, 2>;

  /* ゲインを設定 */
  void SetGain(const Pid::Gain &linear, const Pid::Gain &angular);

  /* 目標値を設定 */
  void SetTarget(float linear, float angular);

  /* リセット */
  void Reset();

  /* 更新 */
  void Update(float batteryVoltage, /* バッテリー電圧 [V] */
              float measureLinear,  /* 速度 [m/s] */
              float measureAccel,   /* 加速度 [m/ss] */
              float measureAngular, /* 角速度 [rad/s] */
              float measureAlpha    /* 角加速度 [rad/ss] */
  );

  /* 設定モーター電圧を取得 */
  Voltage GetMotorVoltage();

  /* デューティ比を取得 */
  Duty GetMotorDuty();

  /* 電圧が丸められたかを取得 */
  bool IsLimited();

 private:
  mutable Mutex mtx_;

  Pid pidLinear_;
  Pid pidAngular_;

  float targetLinear_;
  float targetAngular_;

  Voltage voltage_{};
  Duty duty_{};
  bool isLimited_{false};
};
}  // namespace MotionPlaning
#endif  // MOTIONPLANING_SERVO_H_
