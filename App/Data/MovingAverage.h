#ifndef DATA_MOVINGAVERAGE_H_
#define DATA_MOVINGAVERAGE_H_

/* Project */
#include "RingBuffer.h"

template <typename T, typename U, std::size_t N>
class MovingAverage {
 public:
  MovingAverage() { Reset(); }
  ~MovingAverage() = default;

  void Reset() {
    samples_.Reset();
    sums_ = static_cast<U>(0);
    average_ = static_cast<U>(0);
  }

  U Get() const { return average_; }

  void Update(T sample) {
    if (samples_.Size() == 0) [[unlikely]] {
      sums_ = static_cast<U>(0);
      /* 初回は与えられた値でバッファを満たす */
      for (std::size_t n = 0; n < N; n++) {
        samples_.PushBack(sample);
        sums_ += static_cast<U>(sample);
      }
    } else {
      /* サンプルのうち最も古いデータを取得して削除 */
      const T oldest = samples_.Front();
      samples_.PopFront();
      /* 最も古いデータ分を除去 */
      sums_ -= oldest;
      sums_ += sample;
      samples_.PushBack(sample);
    }

    /* 平均して返す */
    average_ = sums_ / static_cast<U>(N);
  }

 private:
  RingBuffer<T, N> samples_; /* サンプルのリングバッファ */
  U sums_;                   /* 積算値のメモ */
  U average_;                /* 平均値 */
};

#endif  // DATA_MOVINGAVERAGE_H_
