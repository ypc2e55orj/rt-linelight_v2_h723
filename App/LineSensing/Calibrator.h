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
  using Coeff = std::array<float, Adc::kNum>;
  using Max = std::array<uint16_t, Adc::kNum>;
  using Min = std::array<uint16_t, Adc::kNum>;

  explicit Calibrator(Adc &adc) : adc_(adc) {
    for (uint32_t num = 0; num < Adc::kNum; num++) {
      min_[num] = UINT16_MAX;
      max_[num] = 0;
    }
  }

  /* 更新 */
  bool Fetch() {
    if (!adc_.Fetch()) {
      return false;
    }

    /* 各センサーの最大値を取得 */
    for (uint32_t num = 0; num < Adc::kNum; num++) {
      uint16_t raw = adc_.GetRaw(num);
      max_[num] = std::max(max_[num], raw);
      min_[num] = std::min(min_[num], raw);
    }
    return true;
  }

  /* 計算 */
  void Calculate() {
    for (uint32_t num = 0; num < Adc::kNum; num++) {
      coeff_[num] = 1 / static_cast<float>(max_[num] - min_[num]);
    }
  }

  /* 係数を取得 */
  const Coeff &GetCoeff() { return coeff_; }

  /* 最大値を取得 */
  const Max &GetMax() { return max_; }

  /* 最小値を取得 (フォトリフレクタの漏れ電流分) */
  const Min &GetMin() { return min_; }

 private:
  Adc &adc_;
  Coeff coeff_{};
  Max max_{};
  Min min_{};
};
}  // namespace LineSensing

#endif  // LINESENSING_CALIBRATOR_H_
