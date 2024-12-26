#include "LineSensing/LineMarker.h"

/* C++ */
#include <cmath>
#include <mutex>

/* Project */
#include "LineSensing/LineMarkerAdc.h"

namespace LineSensing {
/**
 * MARK: SideMarker
 */

/* リセット */
void SideMarker::Reset() {
  state_ = State::kWaiting;
  count_ = 0;
  detectDistance_ = 0.0f;
  ignoreDistance_ = 0.0f;
  adcAverage_.Reset();
}

/* 更新 */
void SideMarker::Update(uint16_t adVal, float distance) {
  adcAverage_.Update(adVal);
  bool isDetect = adcAverage_.Get() > threshold_;
  switch (state_) {
    case State::kIgnoring:
      /* 交差を検出した場合はセンサー間の距離だけ無視する */
      if (std::abs(distance - ignoreDistance_) > (kLineDistanceFromMarker + kMarkerIgnoreOffset)) {
        state_ = State::kWaiting;
      }
      break;
    case State::kWaiting:
      /* 検出されたら位置を記録 */
      if (isDetect) {
        detectDistance_ = distance;
        state_ = State::kPassing;
      }
      break;
    case State::kPassing:
      /* マーカーの幅がしきい値より小さい場合は無視 */
      if (!isDetect) {
        if (std::abs(distance - detectDistance_) < kMarkerDetectDistance) {
          state_ = State::kWaiting;
        } else {
          state_ = State::kPassed;
          count_++;
        }
      }
      break;
    case State::kPassed:
      /* マーカー検出済み */
      state_ = State::kWaiting;
      break;
  }
}

/* 閾値を設定 */
void SideMarker::SetThreshold(float threshold) { threshold_ = threshold; }

/* 無視開始距離を設定 */
void SideMarker::SetIgnore(float distance) {
  ignoreDistance_ = distance;
  state_ = State::kIgnoring;
}

/**
 * MARK: Marker
 */

/* リセット */
void Marker::Reset() {
  std::scoped_lock<Mutex> lock(mtx_);
  for (uint32_t order = 0; order < MarkerAdc::kNum; order++) {
    markers_[order].Reset();
  }
}

/* 更新 */
bool Marker::Update(float distance) {
  auto &adc = MarkerAdc::Instance();
  if (!adc.Fetch()) {
    return false;
  }
  {
    std::scoped_lock<Mutex> lock(mtx_);
    for (uint32_t order = 0; order < MarkerAdc::kNum; order++) {
      uint16_t raw = adc.GetRaw(order);
      markers_[order].Update(raw, distance);
    }
  }
  return true;
}

/* 閾値を設定 */
void Marker::SetThreshold(const Max &mx) {
  std::scoped_lock<Mutex> lock(mtx_);
  for (uint32_t order = 0; order < MarkerAdc::kNum; order++) {
    markers_[order].SetThreshold(mx[order] * kMarkerDetectThreshold);
  }
}

/* 無視開始距離を設定 */
void Marker::SetIgnore(float distance) {
  std::scoped_lock<Mutex> lock(mtx_);
  for (uint32_t order = 0; order < MarkerAdc::kNum; order++) {
    markers_[order].SetIgnore(distance);
  }
}

/* 状態を取得 */
Marker::State Marker::GetState() const {
  std::scoped_lock<Mutex> lock(mtx_);
  State state{};
  for (uint32_t order = 0; order < MarkerAdc::kNum; order++) {
    state[order] = markers_[order].GetState();
  }
  return state;
}

/* 検出回数を取得 */
Marker::Count Marker::GetCount() const {
  std::scoped_lock<Mutex> lock(mtx_);
  Count count{};
  for (uint32_t order = 0; order < MarkerAdc::kNum; order++) {
    count[order] = markers_[order].GetCount();
  }
  return count;
}

/* 検出開始距離を取得 */
Marker::DetectDistance Marker::GetDetectDistance() const {
  std::scoped_lock<Mutex> lock(mtx_);
  DetectDistance distance{};
  for (uint32_t order = 0; order < MarkerAdc::kNum; order++) {
    distance[order] = markers_[order].GetDetectDistance();
  }
  return distance;
}

/* スタートしたか */
bool Marker::IsStarted() const { return markers_[0].GetCount() > 0; }

/* ゴールしたか */
bool Marker::IsGoaled() const { return markers_[0].GetCount() > 1; }

/* 曲率マーカーがあったか */
bool Marker::IsCurvature() const { return markers_[1].GetState() == SideMarker::State::kPassed; };

/**
 * MARK: Line
 */

/* リセット */
void Line::Reset() {
  std::scoped_lock<Mutex> lock(mtx_);
  state_ = State::kNormal;
  errorAngleAverage_.Reset();
}

/* 更新 */
bool Line::Update(float distance) {
  auto &adc = LineAdc::Instance();
  if (!adc.Fetch()) {
    {
      std::scoped_lock<Mutex> lock(mtx_);
      errorAngleAverage_.Update(0.0f);
    }
    /* TODO: 一定以上取得出来ない場合は State::kNone に設定 */
    return false;
  }
  {
    std::scoped_lock<Mutex> lock(mtx_);
    /* 補正電圧に換算 */
    /* TODO: 電圧に換算する意味がないため補正生値で算出 */
    detectNum_ = 0;
    detectPos_ = 0;
    Value v = {};
    for (uint32_t order = 0; order < LineAdc::kNum; order++) {
      rawLast_[order] = adc.GetRaw(order);
      if (rawLast_[order] > threshold_[order]) {
        detectNum_++;
        detectPos_ |= 1 << order;
      }
      float l = logf(rawLast_[order]) + offset_[order];
      v[order] = std::exp(l) * LineAdc::kAdcReferenceVoltage / static_cast<float>(LineAdc::kAdcMaxValue);
    }
    /* ラインと機体の角度を計算 */
    if (detectNum_ == 0) {
      /* 通常か交差の状態から無反応になった場合 */
      if (state_ == State::kNormal || state_ == State::kCrossPassing || state_ == State::kCrossPassed) {
        /* 無反応開始位置を記録 */
        state_ = State::kNoneDetecting;
        brownOutDistance_ = distance;
      } else if (state_ == State::kNoneDetecting) {
        /* 開始位置から一定以上進んでもラインがない場合 */
        if (std::abs(distance - brownOutDistance_) >= kLineBrownOutIgnoreDistance) {
          /* ラインなし判定 */
          state_ = State::kNone;
        }
      }
    } else if (detectNum_ >= kLineCrossDetectNum) {
      /* マーカーセンサーに交差無視を設定 */
      state_ = State::kCrossPassing;
    } else {
      /* 通常 */
      if (state_ == State::kCrossPassing) {
        state_ = State::kCrossPassed;
      } else {
        state_ = State::kNormal;
      }
      errorAngleAverage_.Update(CalculateErrorAngle(v));
    }
  }
  return true;
}

/* 閾値を設定 */
void Line::SetThreshold(const Max &mx) {
  std::scoped_lock<Mutex> lock(mtx_);
  for (uint32_t order = 0; order < LineAdc::kNum; order++) {
    threshold_[order] = mx[order] * kLineDetectThreshold;
  }
}

/* オフセットを設定 */
void Line::SetOffset(const Offset &of) {
  std::scoped_lock<Mutex> lock(mtx_);
  std::copy(of.begin(), of.end(), offset_.begin());
}

/* オフセットを取得 */
Line::Offset Line::GetOffset() const {
  std::scoped_lock<Mutex> lock(mtx_);
  return offset_;
}

/* 生値を取得 */
void Line::GetRaw(Raw &raw) const {
  std::scoped_lock<Mutex> lock(mtx_);
  raw = rawLast_;
}

/* 状態を取得 */
Line::State Line::GetState() const { return state_; }

/* 反応センサーの個数を取得 */
uint8_t Line::GetDetectNum() const { return detectNum_; }

/* 反応センサーの位置を取得 */
uint16_t Line::GetDetectPos() const { return detectPos_; }

/* エラー角度を取得 */
float Line::GetErrorAngle() const {
  std::scoped_lock<Mutex> lock(mtx_);

  switch (state_) {
    case State::kNoneDetecting:
    case State::kNone:
    case State::kNormal:
      return errorAngleAverage_.Get();
    case State::kCrossPassing:
    default:
      return 0.0f;
  }
}

/* ラインがないか */
bool Line::IsNone() const { return state_ == State::kNone; }

/* 交差か */
bool Line::IsCrossPassed() const { return state_ == State::kCrossPassed; }

/* ラインとのエラー角度を計算 */
float Line::CalculateErrorAngle(const Value &v) {
  float diff = 0.0f;
  for (uint32_t order = 0; order < 8; order++) {
    diff += (v[order] - v[order + 8]) * static_cast<float>(order + 1) / 8.0f;
  }
  return std::atan((kLineToDistCoeff * diff + kLineToDistIntercept) / kLineDistanceFromCenter);
}

}  // namespace LineSensing
