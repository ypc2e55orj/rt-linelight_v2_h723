#ifndef LINESENSING_LINEMARKERADC_H_
#define LINESENSING_LINEMARKERADC_H_

/* STM32CubeMX */
#include <main.h>

/* FreeRTOS */
#include <FreeRTOS.h>
#include <semphr.h>

/* Project */
#include "Config.h"
#include "Data/Singleton.h"
#include "max11128_reg.h"

namespace LineSensing {
/* IR LED点灯 */
void SetIr(bool state);

/**
 * MARK: LineAdc
 */

class LineAdc final : public Singleton<LineAdc> {
 public:
  static constexpr uint32_t kAdcResolution = 12; /* ADC 分解能 */
  static constexpr uint32_t kAdcMaxValue = (1 << kAdcResolution) - 1;
  static constexpr float kAdcReferenceVoltage = kRegulatorVoltage; /* ADC 基準電圧 */
  static constexpr uint32_t kNum = NUM_MAX11128_AIN;

  /* コンストラクタ */
  LineAdc();

  /* 初期化 */
  bool Initialize();

  /* 値を更新 */
  bool Fetch();

  /* 値を取得 */
  uint16_t GetRaw(uint32_t order);

 private:
  /* 外付けADC */
  static void TxRxCpltCallback(SPI_HandleTypeDef *);
  ALIGN_32BYTES(uint16_t rxBuffer_[16]);
  ALIGN_32BYTES(uint16_t txBuffer_[16]);
  StaticSemaphore_t txRxCpltSemphrBuffer_;
  SemaphoreHandle_t txRxCpltSemphr_; /* 送受信完了セマフォ */

  /* MAX11128 のレジスタを設定 */
  void ConfigureMax11128();
};

/**
 * MARK: MarkerAdc
 */

class MarkerAdc final : public Singleton<MarkerAdc> {
 public:
  static constexpr uint32_t kAdcResolution = 12; /* ADC 分解能 */
  static constexpr uint32_t kAdcMaxValue = (1 << kAdcResolution) - 1;
  static constexpr float kAdcReferenceVoltage = kRegulatorVoltage; /* ADC 基準電圧 */
  static constexpr uint32_t kNum = 2;

  /* コンストラクタ */
  MarkerAdc();

  /* 初期化 */
  bool Initialize();

  /* 値を更新 */
  bool Fetch();

  /* 値を取得 */
  uint16_t GetRaw(uint32_t order);

 private:
  /* 内蔵ADC1 */
  static void Adc1ConvCpltCallback(ADC_HandleTypeDef *);
  ALIGN_32BYTES(uint16_t adc1Buffer_[16]);
  StaticSemaphore_t adc1SemphrBuffer_;
  SemaphoreHandle_t adc1Semphr_; /* ADC1完了セマフォ */
};
}  // namespace LineSensing

#endif  // LINESENSING_LINEMARKERADC_H_
