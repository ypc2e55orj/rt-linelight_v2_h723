#ifndef POWERMONITORING_POWERADC_H_
#define POWERMONITORING_POWERADC_H_

/* STM32CubeMX */
#include <main.h>

/* FreeRTOS */
#include <FreeRTOS.h>
#include <semphr.h>

/* Project */
#include "Config.h"
#include "Data/Singleton.h"

namespace PowerMonitoring {
class PowerAdc final : public Singleton<PowerAdc> {
 public:
  static constexpr uint32_t kAdcResolution = 12; /* ADC 分解能 */
  static constexpr uint32_t kAdcMaxValue = (1 << kAdcResolution) - 1;
  static constexpr float kAdcReferenceVoltage = kRegulatorVoltage; /* ADC 基準電圧 */

  enum : uint32_t {
    kOrderMotorCurrentRight,
    kOrderMotorCurrentLeft,
    kOrderBatteryVoltage,
    kNumOrder,
  };

  /* コンストラクタ */
  PowerAdc();

  /* 初期化 */
  bool Initialize();

  /* 値を更新 */
  bool Fetch();

  /* 値を取得 */
  uint16_t GetRaw(uint32_t order);

 private:
  /* ADC2 完了割り込み */
  static void Adc2ConvCpltCallback(ADC_HandleTypeDef *);
  ALIGN_32BYTES(uint16_t adc2Buffer_[16]); /* ADC2転送バッファ */
  StaticSemaphore_t adc2SemphrBuffer_;     /* ADC2完了セマフォバッファ */
  SemaphoreHandle_t adc2Semphr_;           /* ADC2完了セマフォ */
};
}  // namespace PowerMonitoring
#endif  // POWERMONITORING_POWERADC_H_
