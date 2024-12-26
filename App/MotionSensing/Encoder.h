#ifndef MOTIONSENSING_ENCODER_H_
#define MOTIONSENSING_ENCODER_H_

/* C++ */
#include <array>
#include <numbers>

/* Project */
#include "Config.h"
#include "Data/Singleton.h"
#include "Wrapper/Mutex.h"

namespace MotionSensing {
/**
 * MARK: Encoder
 */
class Encoder final : public Singleton<Encoder> {
 public:
  static constexpr float kWheelPulsePerRev = kGearRatio * (4.0f * 1024.0f); /* 車輪1回転あたりのパルス数 */
  static constexpr float kAnglePerPulse = 2.0f * std::numbers::pi_v<float> / kWheelPulsePerRev;

  using Count = std::array<uint16_t, 2>;
  using Diff = std::array<float, 2>;

  /* 初期化 */
  bool Initialize();

  /* リセット */
  void Reset();

  /* 更新 */
  void Update();

  /* カウント値を取得 */
  Count GetCount();

  /* 車輪変化角度を取得 [rad] */
  Diff GetDiff();

 private:
  static constexpr uint16_t kTimMaxValue = UINT16_MAX;      /* カウント値保持レジスタの分解能 */
  static constexpr uint16_t kTimHalfValue = UINT16_MAX / 2; /* カウント値保持レジスタの分解能(半分) */

  Mutex mtx_;
  Count last_{};
  Diff diff_{};

  /* カウント値から車輪変化角度を算出 */
  static int32_t CalcWheelDelta(uint16_t curr, uint16_t prev);
};
}  // namespace MotionSensing
#endif  // MOTIONSENSING_ENCODER_H_
