#include "PowerMonitoring/PowerAdc.h"

/* グローバル変数定義 */
extern ADC_HandleTypeDef hadc2;

namespace PowerMonitoring {
/* 内蔵ADC2 変換完了コールバック */
void PowerAdc::Adc2ConvCpltCallback(ADC_HandleTypeDef *) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(PowerAdc::Instance().adc2Semphr_, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* コンストラクタ */
PowerAdc::PowerAdc() : adc2Semphr_(xSemaphoreCreateBinaryStatic(&adc2SemphrBuffer_)) {}

/* 初期化 */
bool PowerAdc::Initialize() {
  return HAL_ADC_RegisterCallback(&hadc2, HAL_ADC_CONVERSION_COMPLETE_CB_ID, Adc2ConvCpltCallback) == HAL_OK;
}

/* 値を更新 */
bool PowerAdc::Fetch() {
  if (HAL_ADC_Start_DMA(&hadc2, reinterpret_cast<uint32_t *>(adc2Buffer_), kNumOrder) == HAL_OK) {
    /* TODO: タイムアウト */
    if (xSemaphoreTake(adc2Semphr_, pdMS_TO_TICKS(1)) == pdTRUE) {
      if (HAL_ADC_Stop_DMA(&hadc2) == HAL_OK) {
        /* キャッシュラインを更新 */
        SCB_CleanInvalidateDCache_by_Addr((uint32_t *)adc2Buffer_, sizeof(adc2Buffer_));
        return true;
      }
    }
  }
  return false;
}

/* 値を取得 */
uint16_t PowerAdc::GetRaw(uint32_t order) { return adc2Buffer_[order]; }
}  // namespace PowerMonitoring
