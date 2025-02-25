#ifndef LINESENSING_LINE_H_
#define LINESENSING_LINE_H_

/* STM32CubeMX */
#include <main.h>

/* C++ */
#include <algorithm>
#include <array>

/* FreeRTOS */
#include <FreeRTOS.h>
#include <semphr.h>

/* Project */
#include "Config.h"
#include "Data/MovingAverage.h"
#include "Data/Singleton.h"
#include "Wrapper/Mutex.h"
#include "max11128_reg.h"

namespace LineSensing {

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
  inline uint16_t GetRaw(uint32_t order) { return rxBuffer_[order] & 0x0fff; }

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

class LineImpl {
 public:
  static constexpr uint32_t kNum = 16;

  enum class State {
    kNoneDetecting, /* コースアウト検出中 */
    kNone,          /* コースアウト */
    kNormal,        /* 通常 */
    kCrossPassing,  /* 交差通過中 */
    kCrossPassed,   /* 交差通過完了 */
  };

  /* リセット */
  void Reset();

  /* 更新 */
  bool Update(float distance);

  /* キャリブレーション値を設定 */
  void SetCalibration(const std::array<uint16_t, kNum> &min, /* 最小値 */
                      const std::array<uint16_t, kNum> &max, /* 最大値 */
                      const std::array<float, kNum> &coeff   /* 係数 */
  );

  /* 生値を取得 */
  std::array<uint16_t, kNum> GetRaw() const;

  /* 状態を取得 */
  State GetState() const;

  /* 反応センサーの個数を取得 */
  uint8_t GetDetectNum() const;

  /* エラーを取得 */
  float GetError() const;

  /* ラインがないか */
  bool IsNone() const;

  /* 交差か */
  bool IsCrossPassed() const;

 private:
  mutable Mutex mtx_;

  std::array<float, kNum> coeff_;
  std::array<uint16_t, kNum> min_;
  std::array<uint16_t, kNum> max_;

  State state_;                                           /* 前回の状態 */
  uint8_t detectNum_;                                     /* 反応センサーの個数 */
  float brownOutDistance_;                                /* ライン無反応開始距離 [m] */
  MovingAverage<float, float, kLineNumErrorMovingAverage> /* エラーの移動平均 */
      errorAverage_;
};
}  // namespace LineSensing

#endif  // LINESENSING_LINE_H_
