#include "LineSensing/Line.h"

/* C++ */
#include <cmath>
#include <cstring>
#include <mutex>

/* Project */
#include "LineSensing/Line.h"

/* グローバル変数定義 */
extern SPI_HandleTypeDef hspi3;

namespace LineSensing {
/* MAX11128 送受信コールバック */
void LineAdc::TxRxCpltCallback(SPI_HandleTypeDef *) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(LineAdc::Instance().txRxCpltSemphr_, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* MAX11128 のレジスタを設定 */
void LineAdc::ConfigureMax11128() {
  /* MAX11128の初期化 */
  static const uint16_t txSampleSet[4] = {
      // clang-format off
                               /* r0               r1               r2               r3 */
      MAX11128_TO_SAMPLESET_FRAME(MAX11128_AIN_8,  MAX11128_AIN_9,  MAX11128_AIN_10, MAX11128_AIN_11),
                               /* r4               r5               r6               r7 */
      MAX11128_TO_SAMPLESET_FRAME(MAX11128_AIN_12, MAX11128_AIN_13, MAX11128_AIN_14, MAX11128_AIN_15),
                               /* l0               l1               l2               l3 */
      MAX11128_TO_SAMPLESET_FRAME(MAX11128_AIN_7,  MAX11128_AIN_6,  MAX11128_AIN_5,  MAX11128_AIN_4),
                               /* l4               l5               l6               l7 */
      MAX11128_TO_SAMPLESET_FRAME(MAX11128_AIN_3,  MAX11128_AIN_2,  MAX11128_AIN_1,  MAX11128_AIN_0),
      // clang-format on
  };
  MAX11128_REG reg = {0};

  reg.raw = 0;
  reg.ctrl.reset = MAX11128_ADC_MODE_CTRL_RESET_ALL;
  HAL_SPI_Transmit(&hspi3, (uint8_t *)&reg, 1, HAL_MAX_DELAY);

  reg.raw = 0;
  reg.smpl_set.smpl_set = MAX11128_REG_IDENT_SMPL_SET;
  reg.smpl_set.seq_length = 15;
  HAL_SPI_Transmit(&hspi3, (uint8_t *)&reg, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&hspi3, (uint8_t *)&txSampleSet, 4, HAL_MAX_DELAY);

  reg.raw = 0;
  reg.ctrl.scan = MAX11128_ADC_MODE_CTRL_SCAN_SAMPLESET;
  reg.ctrl.chan_id = 1;
  reg.ctrl.chsel = MAX11128_AIN_15;
  HAL_SPI_Transmit(&hspi3, (uint8_t *)&reg, 1, HAL_MAX_DELAY);

  /* 送信バッファを設定 */
  std::memset(txBuffer_, 0, sizeof(txBuffer_));
  SCB_CleanDCache_by_Addr(reinterpret_cast<uint32_t *>(txBuffer_), sizeof(txBuffer_));
}

/* コンストラクタ */
LineAdc::LineAdc() : txRxCpltSemphr_(xSemaphoreCreateBinaryStatic(&txRxCpltSemphrBuffer_)) {}

/* 初期化 */
bool LineAdc::Initialize() {
  ConfigureMax11128();
  return HAL_SPI_RegisterCallback(&hspi3, HAL_SPI_TX_RX_COMPLETE_CB_ID, TxRxCpltCallback) == HAL_OK;
}

/* 値を更新 */
bool LineAdc::Fetch() {
  for (int i = 0; i < NUM_MAX11128_AIN; i++) {
    /* MAX11128はバースト転送出来ないようなので1チャンネルずつ読み出し */
    const uint8_t *pTx = reinterpret_cast<const uint8_t *>(&txBuffer_[i]);
    uint8_t *pRx = reinterpret_cast<uint8_t *>(&rxBuffer_[i]);
    if (HAL_SPI_TransmitReceive_DMA(&hspi3, pTx, pRx, 1) != HAL_OK) {
      return false;
    }
    if (xSemaphoreTake(txRxCpltSemphr_, pdMS_TO_TICKS(1)) == pdFALSE) {
      return false;
    }
  }
  /* キャッシュラインを更新 */
  SCB_CleanInvalidateDCache_by_Addr((uint32_t *)rxBuffer_, sizeof(rxBuffer_));
  return true;
}

/* リセット */
void LineImpl::Reset() {
  std::scoped_lock<Mutex> lock(mtx_);
  state_ = State::kNormal;
  errorAverage_.Reset();
}

/* 更新 */
bool LineImpl::Update(float distance) {
  auto &adc = LineAdc::Instance();
  if (!adc.Fetch()) {
    {
      std::scoped_lock<Mutex> lock(mtx_);
      errorAverage_.Update(0.0f);
    }
    return false;
  }
  {
    std::scoped_lock<Mutex> lock(mtx_);
    /* ラインセンサーの値を補正、反応個数を計算 */
    std::array<float, kNum> value{};
    detectNum_ = 0;
    for (uint32_t order = 0; order < LineAdc::kNum; order++) {
      uint16_t val = std::clamp(adc.GetRaw(order), min_[order], max_[order]);
      if (val > max_[order] * kLineDetectThreshold) {
        detectNum_++;
      }
      value[order] = coeff_[order] * (val - min_[order]);
    }
    /* ラインセンサーの値を一次元化 */
    float diff = 0.0f;
    for (uint32_t order = 0; order < 8; order++) {
      diff += (value[order] - value[order + 8]) * static_cast<float>(order + 1) / 8.0f;
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
      errorAverage_.Update(diff);
    }
  }
  return true;
}

/* キャリブレーション値を設定 */
void LineImpl::SetCalibration(const std::array<uint16_t, kNum> &min, /* 最小値 */
                              const std::array<uint16_t, kNum> &max, /* 最大値 */
                              const std::array<float, kNum> &coeff   /* 係数 */
) {
  std::scoped_lock<Mutex> lock(mtx_);
  min_ = min;
  max_ = max;
  coeff_ = coeff;
}

/* 生値を取得 */
std::array<uint16_t, LineImpl::kNum> LineImpl::GetRaw() const {
  std::scoped_lock<Mutex> lock(mtx_);
  auto &adc = LineAdc::Instance();
  std::array<uint16_t, kNum> raw{};
  for (uint32_t order = 0; order < LineAdc::kNum; order++) {
    raw[order] = adc.GetRaw(order);
  }
  return raw;
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
