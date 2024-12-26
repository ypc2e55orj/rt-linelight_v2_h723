#include "LineSensing/LineMarkerAdc.h"

/* Project */
#include "Config.h"
#include "LineSensing/max11128_reg.h"

/* C++ */
#include <cstdio>
#include <cstring>

/* グローバル変数定義 */
extern SPI_HandleTypeDef hspi3;
extern ADC_HandleTypeDef hadc1;

namespace LineSensing {
/* IR LED点灯 */
void SetIr(bool state) {
  GPIO_InitTypeDef gpioPrOff = {};
  gpioPrOff.Pin = IRLED_OFF_Pin;
  gpioPrOff.Pull = state ? GPIO_PULLDOWN : GPIO_NOPULL;
  HAL_GPIO_Init(IRLED_OFF_GPIO_Port, &gpioPrOff);
}

/**
 * MARK: LineAdc
 */

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

/* 値を取得 */
uint16_t LineAdc::GetRaw(uint32_t order) { return rxBuffer_[order] & 0x0fff; }

/**
 * MARK: MarkerAdc
 */

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
}  // namespace LineSensing
