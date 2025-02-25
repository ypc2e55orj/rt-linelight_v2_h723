#ifndef LINESENSING_MARKER_H_
#define LINESENSING_MARKER_H_

/* STM32CubeMX */
#include <main.h>

/* C++ */
#include <array>

/* FreeRTOS */
#include <FreeRTOS.h>
#include <semphr.h>

/* Project */
#include "Config.h"
#include "Data/MovingAverage.h"
#include "Data/Singleton.h"
#include "Wrapper/Mutex.h"

namespace LineSensing {

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

class MarkerImpl {
 public:
  static constexpr uint32_t kNum = 2;

  enum class State {
    kIgnoring, /* マーカー無視中 */
    kWaiting,  /* マーカー待ち */
    kPassing,  /* マーカー通過中 */
    kPassed,   /* マーカー通過完了 */
  };

  /* リセット */
  void Reset();

  /* 更新 */
  bool Update(float distance);

  /* 閾値を設定 */
  void SetCalibration(const std::array<uint16_t, kNum> &max);

  /* 無視開始距離を設定 */
  void SetIgnore(float distance);

  /* 状態を取得 */
  std::array<State, kNum> GetState() const;

  /* 検知回数を取得 */
  std::array<uint32_t, kNum> GetCount() const;

  /* スタートしたか */
  bool IsStarted() const;

  /* ゴールしたか */
  bool IsGoaled() const;

  /* 曲率マーカーがあったか */
  bool IsCurvature() const;

 private:
  using Average = MovingAverage<uint16_t, uint16_t, kMarkerNumMovingAverage>;

  mutable Mutex mtx_;

  std::array<Average, kNum> average_;              /* ADC移動平均 */
  std::array<State, kNum> state_{State::kWaiting}; /* 前回の状態 */
  std::array<float, kNum> threshold_;              /* 検出閾値 */
  std::array<uint32_t, kNum> count_;               /* 検知回数 */
  std::array<float, kNum> detectDistance_;         /* 検出開始距離 [m] */
  float ignoreDistance_;                           /* 無視開始距離 [m] */
};
}  // namespace LineSensing
#endif  // LINESENSING_MARKER_H_
