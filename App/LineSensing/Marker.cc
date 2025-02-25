#include "LineSensing/Marker.h"

/* C++ */
#include <mutex>

/* グローバル変数定義 */
extern ADC_HandleTypeDef hadc1;

namespace LineSensing {
/* 内蔵ADC1 変換完了コールバック */
void MarkerAdc::Adc1ConvCpltCallback(ADC_HandleTypeDef *) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(MarkerAdc::Instance().adc1Semphr_, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* コンストラクタ */
MarkerAdc::MarkerAdc() : adc1Semphr_(xSemaphoreCreateBinaryStatic(&adc1SemphrBuffer_)) {}

/* 初期化 */
bool MarkerAdc::Initialize() {
  return HAL_ADC_RegisterCallback(&hadc1, HAL_ADC_CONVERSION_COMPLETE_CB_ID, Adc1ConvCpltCallback) == HAL_OK;
}

/* 値を更新 */
bool MarkerAdc::Fetch() {
  /* AD変換開始 */
  if (HAL_ADC_Start_DMA(&hadc1, reinterpret_cast<uint32_t *>(adc1Buffer_), kNum) == HAL_OK) {
    /* TODO: タイムアウト */
    if (xSemaphoreTake(adc1Semphr_, pdMS_TO_TICKS(1)) == pdTRUE) {
      if (HAL_ADC_Stop_DMA(&hadc1) == HAL_OK) {
        /* キャッシュラインを更新 */
        SCB_CleanInvalidateDCache_by_Addr((uint32_t *)adc1Buffer_, sizeof(adc1Buffer_));
        return true;
      }
    }
  }
  return false;
}

/* 値を取得 */
uint16_t MarkerAdc::GetRaw(uint32_t order) { return adc1Buffer_[order] & 0x0fff; }

/* リセット */
void MarkerImpl::Reset() {
  std::scoped_lock<Mutex> lock(mtx_);
  ignoreDistance_ = 0.0f;
  for (uint32_t order = 0; order < MarkerAdc::kNum; order++) {
    state_[order] = State::kWaiting;
    count_[order] = 0;
    detectDistance_[order] = 0.0f;
    average_[order].Reset();
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

      average_[order].Update(raw);
      bool isDetect = average_[order].Get() > threshold_[order];
      switch (state_[order]) {
        case State::kIgnoring:
          /* 交差を検出した場合はセンサー間の距離だけ無視する */
          if (std::abs(distance - ignoreDistance_) > (kLineDistanceFromMarker + kMarkerIgnoreOffset)) {
            state_[order] = State::kWaiting;
          }
          break;
        case State::kWaiting:
          /* 検出されたら位置を記録 */
          if (isDetect) {
            detectDistance_[order] = distance;
            state_[order] = State::kPassing;
          }
          break;
        case State::kPassing:
          /* マーカーの幅がしきい値より小さい場合は無視 */
          if (!isDetect) {
            if (std::abs(distance - detectDistance_[order]) < kMarkerDetectDistance) {
              state_[order] = State::kWaiting;
            } else {
              state_[order] = State::kPassed;
              count_[order]++;
            }
          }
          break;
        case State::kPassed:
          /* マーカー検出済み */
          state_[order] = State::kWaiting;
          break;
      }
    }
  }
  return true;
}

/* 閾値を設定 */
void MarkerImpl::SetCalibration(const std::array<uint16_t, kNum> &max) {
  std::scoped_lock<Mutex> lock(mtx_);
  for (uint32_t order = 0; order < MarkerAdc::kNum; order++) {
    threshold_[order] = max[order] * kMarkerDetectThreshold;
  }
}

/* 無視開始距離を設定 */
void MarkerImpl::SetIgnore(float distance) {
  std::scoped_lock<Mutex> lock(mtx_);
  ignoreDistance_ = distance;
  for (uint32_t order = 0; order < MarkerAdc::kNum; order++) {
    state_[order] = State::kIgnoring;
  }
}

/* 状態を取得 */
std::array<MarkerImpl::State, MarkerImpl::kNum> MarkerImpl::GetState() const {
  std::scoped_lock<Mutex> lock(mtx_);
  return state_;
}

/* 検知回数を取得 */
std::array<uint32_t, MarkerImpl::kNum> MarkerImpl::GetCount() const {
  std::scoped_lock<Mutex> lock(mtx_);
  return count_;
}

/* スタートしたか */
bool MarkerImpl::IsStarted() const { return count_[0] > 0; }

/* ゴールしたか */
bool MarkerImpl::IsGoaled() const { return count_[0] > 1; }

/* 曲率マーカーがあったか */
bool MarkerImpl::IsCurvature() const { return state_[1] == State::kPassed; };
}  // namespace LineSensing
