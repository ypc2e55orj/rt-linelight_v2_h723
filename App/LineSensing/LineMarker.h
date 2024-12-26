#ifndef LINESENSING_LINEMARKER_H_
#define LINESENSING_LINEMARKER_H_

/* C++ */
#include <algorithm>
#include <array>

/* Project */
#include "Config.h"
#include "Data/MovingAverage.h"
#include "Wrapper/Mutex.h"

namespace LineSensing {
/**
 * MARK: SideMarker
 */

class SideMarker {
 public:
  enum class State {
    kIgnoring, /* マーカー無視中 */
    kWaiting,  /* マーカー待ち */
    kPassing,  /* マーカー通過中 */
    kPassed,   /* マーカー通過完了 */
  };

  /* リセット */
  void Reset();

  /* 更新 */
  void Update(uint16_t adVal, float distance);

  /* 閾値を設定 */
  void SetThreshold(float threshold);

  /* 無視開始距離を設定 */
  void SetIgnore(float distance);

  /* 状態を取得 */
  State GetState() const { return state_; }

  /* 検出回数を取得 */
  uint32_t GetCount() const { return count_; }

  /* 検出開始距離を取得 */
  float GetDetectDistance() const { return detectDistance_; }

 private:
  float threshold_; /* マーカー検出閾値 */

  State state_{State::kWaiting}; /* 前回の状態 */
  uint32_t count_;               /* マーカーの検知回数 */
  float detectDistance_;         /* マーカー検出開始距離 [m] */
  float ignoreDistance_;         /* マーカー無視開始距離 [m] */
  MovingAverage<uint16_t, uint16_t, kMarkerNumMovingAverage> adcAverage_;
};

/**
 * MARK: Marker
 */

class Marker {
 public:
  static constexpr uint32_t kNum = 2;

  using Max = std::array<uint16_t, kNum>;
  using State = std::array<SideMarker::State, kNum>;
  using Count = std::array<uint32_t, kNum>;
  using DetectDistance = std::array<float, kNum>;

  /* リセット */
  void Reset();

  /* 更新 */
  bool Update(float distance);

  /* 閾値を設定 */
  void SetThreshold(const Max &mx);

  /* 無視開始距離を設定 */
  void SetIgnore(float distance);

  /* 状態を取得 */
  State GetState() const;

  /* 検出回数を取得 */
  Count GetCount() const;

  /* 検出開始距離を取得 */
  DetectDistance GetDetectDistance() const;

  /* スタートしたか */
  bool IsStarted() const;

  /* ゴールしたか */
  bool IsGoaled() const;

  /* 曲率マーカーがあったか */
  bool IsCurvature() const;

 private:
  mutable Mutex mtx_;
  std::array<SideMarker, kNum> markers_;
};

/**
 * MARK: Line
 */
class Line {
 public:
  static constexpr uint32_t kNum = 16;

  using Offset = std::array<float, kNum>;
  using Max = std::array<uint16_t, kNum>;
  using Raw = std::array<uint16_t, kNum>;

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

  /* 閾値を設定 */
  void SetThreshold(const Max &mx);

  /* オフセットを設定 */
  void SetOffset(const Offset &of);

  /* オフセットを取得 */
  Offset GetOffset() const;

  /* 生値を取得 */
  void GetRaw(Raw &raw) const;

  /* 状態を取得 */
  State GetState() const;

  /* 反応センサーの個数を取得 */
  uint8_t GetDetectNum() const;

  /* 反応センサーの位置を取得 */
  uint16_t GetDetectPos() const;

  /* エラー角度を取得 */
  float GetErrorAngle() const;

  /* ラインがないか */
  bool IsNone() const;

  /* 交差か */
  bool IsCrossPassed() const;

 private:
  using Value = std::array<float, 16>;
  mutable Mutex mtx_;

  /* ラインとのエラー角度を計算 */
  static float CalculateErrorAngle(const Value &v);

  Value threshold_; /* 閾値 */
  Offset offset_;   /* オフセット */
  Raw rawLast_;     /* 最新の生値 */

  /* 加工後の状態 */
  State state_;
  uint16_t detectPos_;                                    /* 反応センサーの位置 [bit] */
  uint8_t detectNum_;                                     /* 反応センサーの個数 */
  float brownOutDistance_;                                /* ライン無反応開始距離 [m] */
  MovingAverage<float, float, kLineNumErrorMovingAverage> /* エラー角度の移動平均 */
      errorAngleAverage_;
};
}  // namespace LineSensing

#endif  // LINESENSING_LINEMARKER_H_
