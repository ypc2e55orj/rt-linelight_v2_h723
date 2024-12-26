#include "PowerMonitoring/Power.h"

/* C++ */
#include <mutex>

/* Project */
#include "PowerMonitoring/PowerAdc.h"

namespace PowerMonitoring {
/* リセット */
void Power::Reset() {
  std::scoped_lock<Mutex> lock(mtx_);
  batteryVoltageMovingAverage_.Reset();
  batteryVoltage_ = 0.0f;
  motorCurrent_ = {0.0f, 0.0f};
  batteryErrorCount_ = 0;
  prevTick_ = HAL_GetTick();
}

/* 更新 */
bool Power::Update() {
  auto &adc = PowerAdc::Instance();
  std::array<float, 3> voltage{};
  if (!adc.Fetch()) {
    {
      std::scoped_lock<Mutex> lock(mtx_);
      adcErrorCount_++;
      batteryVoltageMovingAverage_.Update(0.0f);
      motorCurrent_ = {0.0f, 0.0f};
    }
    return false;
  }
  /* 電圧に換算 */
  for (uint32_t order = 0; order < 3; order++) {
    std::scoped_lock<Mutex> lock(mtx_);
    voltage[order] = (adc.GetRaw(order) * PowerAdc::kAdcReferenceVoltage) / PowerAdc::kAdcMaxValue;
  }
  {
    std::scoped_lock<Mutex> lock(mtx_);
    auto tick = HAL_GetTick();
    diffTick_ = tick - prevTick_;
    adcErrorCount_ = 0;
    batteryVoltage_ = voltage[PowerAdc::kOrderBatteryVoltage] * kBatteryVoltageAdcGain;
    batteryVoltageMovingAverage_.Update(batteryVoltage_);

    if (kBatteryVoltageLimitMin < batteryVoltageMovingAverage_.Get()) {
      batteryErrorCount_ = 0;
    } else {
      batteryErrorCount_++;
    }
    float right = (2.0f * voltage[PowerAdc::kOrderMotorCurrentRight] - kRegulatorVoltage) /
                  (kMotorCurrentMeasureDivResistor / 10000.0f);
    float left = (2.0f * voltage[PowerAdc::kOrderMotorCurrentLeft] - kRegulatorVoltage) /
                 (kMotorCurrentMeasureDivResistor / 10000.0f);
    motorCurrent_ = {right, left};
    prevTick_ = tick;
  }
  return true;
}

/* 電池電圧を取得 */
float Power::GetBatteryVoltage() const { return batteryVoltage_; }

/* 電池電圧を取得 */
float Power::GetBatteryVoltageAverage() const { return batteryVoltageMovingAverage_.Get(); }

/* モーター電流を取得 */
Power::MotorCurrent Power::GetMotorCurrent() const {
  std::scoped_lock<Mutex> lock(mtx_);
  return motorCurrent_;
}

/* 電池エラー連続時間 [ms] を取得 */
uint32_t Power::GetBatteryErrorTime() const { return batteryErrorCount_; }

/* ADCエラー連続時間 [ms] を取得 */
uint32_t Power::GetAdcErrorTime() const { return adcErrorCount_; }

/* 更新周期を取得 */
uint32_t Power::GetTick() const { return diffTick_; }
}  // namespace PowerMonitoring
