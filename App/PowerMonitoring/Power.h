#ifndef POWERMONITORING_POWER_H_
#define POWERMONITORING_POWER_H_

/* Projects */
#include "Config.h"
#include "Data/MovingAverage.h"
#include "Wrapper/Mutex.h"

/* C++ */
#include <array>

namespace PowerMonitoring {
class Power {
 public:
  using MotorCurrent = std::array<float, 2>;

  /* リセット */
  void Reset();

  /* 更新 */
  bool Update();

  /* 電池電圧を取得 */
  float GetBatteryVoltage() const;

  /* 電池電圧を取得(移動平均) */
  float GetBatteryVoltageAverage() const;

  /* モーター電流を取得 */
  MotorCurrent GetMotorCurrent() const;

  /* 電池エラー連続時間 [ms] を取得 */
  uint32_t GetBatteryErrorTime() const;

  /* ADCエラー連続時間 [ms] を取得 */
  uint32_t GetAdcErrorTime() const;

  /* 更新周期を取得 */
  TickType_t GetTick() const;

 private:
  mutable Mutex mtx_;

  uint32_t adcErrorCount_{0};                                  /* ADC取得エラーカウント */
  MovingAverage<float, float, kBatteryVoltageNumMovingAverage> /* 電池電圧の移動平均 [V] */
      batteryVoltageMovingAverage_{};
  float batteryVoltage_{0.0f};            /* 電池電圧 [V] */
  uint32_t batteryErrorCount_{0};         /* 連続異常カウント */
  MotorCurrent motorCurrent_{0.0f, 0.0f}; /* 計測モーター電流 [A] */
  uint32_t prevTick_{0};
  uint32_t diffTick_{0};
};
}  // namespace PowerMonitoring
#endif  // POWERMONITORING_POWER_H_
