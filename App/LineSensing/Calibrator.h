#ifndef LINESENSING_CALIBRATOR_H_
#define LINESENSING_CALIBRATOR_H_

#include <concepts>
#include <cstdint>

namespace LineSensing {

/**
 * MARK: AdcClass
 */

template <typename T>
concept AdcClass = requires(T &t) {
  T::kNum;
  { t.Fetch() } -> std::same_as<bool>;
  { t.GetRaw(std::declval<uint32_t>()) } -> std::same_as<uint16_t>;
};

/**
 * MARK: Calibrator
 */

template <AdcClass Adc>
class Calibrator {
 public:
  using Offset = std::array<float, Adc::kNum>;
  using Max = std::array<uint16_t, Adc::kNum>;

  explicit Calibrator(Adc &adc) : adc_(adc) {}

  /* 更新 */
  bool Fetch() {
    std::array<uint16_t, Adc::kNum> sortBuffer = {};

    if (!adc_.Fetch()) {
      return false;
    }

    /* 各センサーの最大値を取得 */
    for (uint32_t num = 0; num < Adc::kNum; num++) {
      uint16_t raw = adc_.GetRaw(num);
      max_[num] = std::max(max_[num], raw);
      sortBuffer[num] = raw;
    }
    /* 最大の中央値を算出 */
    std::sort(sortBuffer.begin(), sortBuffer.end());
    float median = Adc::kNum % 2 == 0 ? (sortBuffer[Adc::kNum / 2 - 1] + sortBuffer[Adc::kNum / 2]) / 2.0f
                                      : sortBuffer[Adc::kNum / 2];
    medianMax_ = std::max(medianMax_, median);

    return true;
  }

  /* 計算 */
  void Calculate() {
    for (uint32_t num = 0; num < Adc::kNum; num++) {
      offset_[num] = std::log(static_cast<float>(max_[num]) / medianMax_);
    }
  }

  /* 最大値を取得 */
  const Max &GetMax() { return max_; }

  /* オフセットを取得 */
  const Offset &GetOffset() { return offset_; }

 private:
  Adc &adc_;
  float medianMax_{};
  Max max_{};
  Offset offset_{};
};
}  // namespace LineSensing

#endif  // LINESENSING_CALIBRATOR_H_
