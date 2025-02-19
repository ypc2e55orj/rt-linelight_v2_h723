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
void MarkerImpl::Reset() {
  std::scoped_lock<Mutex> lock(mtx_);
  for (uint32_t order = 0; order < MarkerAdc::kNum; order++) {
    markers_[order].Reset();
  }
}

/* 更新 */
bool MarkerImpl::Update(float distance) {
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
void MarkerImpl::SetCalibration(const Max &max) {
  std::scoped_lock<Mutex> lock(mtx_);
  for (uint32_t order = 0; order < MarkerAdc::kNum; order++) {
    markers_[order].SetThreshold(static_cast<float>(max[order]) * kMarkerDetectThreshold);
  }
}

/* 無視開始距離を設定 */
void MarkerImpl::SetIgnore(float distance) {
  std::scoped_lock<Mutex> lock(mtx_);
  for (uint32_t order = 0; order < MarkerAdc::kNum; order++) {
    markers_[order].SetIgnore(distance);
  }
}

/* 状態を取得 */
MarkerImpl::State MarkerImpl::GetState() const {
  std::scoped_lock<Mutex> lock(mtx_);
  State state{};
  for (uint32_t order = 0; order < MarkerAdc::kNum; order++) {
    state[order] = markers_[order].GetState();
  }
  return state;
}

/* 検出回数を取得 */
MarkerImpl::Count MarkerImpl::GetCount() const {
  std::scoped_lock<Mutex> lock(mtx_);
  Count count{};
  for (uint32_t order = 0; order < MarkerAdc::kNum; order++) {
    count[order] = markers_[order].GetCount();
  }
  return count;
}

/* 検出開始距離を取得 */
MarkerImpl::DetectDistance MarkerImpl::GetDetectDistance() const {
  std::scoped_lock<Mutex> lock(mtx_);
  DetectDistance distance{};
  for (uint32_t order = 0; order < MarkerAdc::kNum; order++) {
    distance[order] = markers_[order].GetDetectDistance();
  }
  return distance;
}

/* スタートしたか */
bool MarkerImpl::IsStarted() const { return markers_[0].GetCount() > 0; }

/* ゴールしたか */
bool MarkerImpl::IsGoaled() const { return markers_[0].GetCount() > 1; }

/* 曲率マーカーがあったか */
bool MarkerImpl::IsCurvature() const { return markers_[1].GetState() == SideMarker::State::kPassed; };

/**
 * MARK: Line
 */

/* リセット */
void LineImpl::Reset() {
  std::scoped_lock<Mutex> lock(mtx_);
  state_ = State::kNormal;
  errorAverage_.Reset();
}

/* 更新 */
bool LineImpl::Update(float distance) {
  std::scoped_lock<Mutex> lock(mtx_);
  auto &adc = LineAdc::Instance();
  if (!adc.Fetch()) {
    {
      errorAverage_.Update(0.0f);
    }
    return false;
  }
  /* ラインセンサーの値を補正、反応個数を計算 */
  detectNum_ = 0;
  for (uint32_t order = 0; order < LineAdc::kNum; order++) {
    uint16_t val = std::clamp(adc.GetRaw(order), min_[order], max_[order]);
    if (val > max_[order] * kLineDetectThreshold) {
      detectNum_++;
    }
    valueLast_[order] = coeff_[order] * (val - min_[order]);
  }
  /* ラインセンサーの値を一次元化 */
  diffLast_ = 0.0f;
  for (uint32_t order = 0; order < 8; order++) {
    diffLast_ += (valueLast_[order] - valueLast_[order + 8]) * static_cast<float>(order + 1) / 8.0f;
  }
  /* ライン状態を更新 */
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
    errorAverage_.Update(diffLast_);
  }

  return true;
}

/* キャリブレーション値を設定 */
void LineImpl::SetCalibration(const Min &min, const Max &max, const Coeff &coeff) {
  std::scoped_lock<Mutex> lock(mtx_);
  min_ = min;
  max_ = max;
  coeff_ = coeff;
}

/* 生値を取得 */
void LineImpl::GetRaw(Raw &raw) const {
  std::scoped_lock<Mutex> lock(mtx_);
  auto &adc = LineAdc::Instance();
  for (uint32_t order = 0; order < LineAdc::kNum; order++) {
    raw[order] = adc.GetRaw(order);
  }
}

/* 状態を取得 */
LineImpl::State LineImpl::GetState() const { return state_; }

/* 反応センサーの個数を取得 */
uint8_t LineImpl::GetDetectNum() const { return detectNum_; }

/* エラー角度を取得 */
float LineImpl::GetError() const {
  std::scoped_lock<Mutex> lock(mtx_);

  switch (state_) {
    case State::kNoneDetecting:
    case State::kNone:
    case State::kNormal:
      return errorAverage_.Get();
    case State::kCrossPassing:
    default:
      return 0.0f;
  }
}

/* ラインがないか */
bool LineImpl::IsNone() const { return state_ == State::kNone; }

/* 交差か */
bool LineImpl::IsCrossPassed() const { return state_ == State::kCrossPassed; }
}  // namespace LineSensing
