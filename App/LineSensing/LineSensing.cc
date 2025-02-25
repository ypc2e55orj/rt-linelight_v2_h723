#include "LineSensing/LineSensing.h"

/* Projects */
#include "Config.h"
#include "MotionSensing/MotionSensing.h"
#include "NonVolatileData.h"
#include "Periodic.h"

/* グローバル変数定義 */
extern TIM_HandleTypeDef htim7;

namespace LineSensing {
/* IR LED点灯待ち */
void LineSensing::PeriodElapsedCallback(TIM_HandleTypeDef *) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(LineSensing::Instance().periodElapsedSemphr_, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
bool LineSensing::TurnOnIrLed() {
  HAL_GPIO_WritePin(IR_EN_GPIO_Port, IR_EN_Pin, GPIO_PIN_SET);
  return HAL_TIM_Base_Start_IT(&htim7) == HAL_OK &&                          /* 立ち上がり待ちタイマー開始 */
         xSemaphoreTake(periodElapsedSemphr_, pdMS_TO_TICKS(1)) == pdTRUE && /* カウント待ち */
         HAL_TIM_Base_Stop_IT(&htim7) == HAL_OK;
}
void LineSensing::TurnOffIrLed() { HAL_GPIO_WritePin(IR_EN_GPIO_Port, IR_EN_Pin, GPIO_PIN_RESET); }

LineSensing::LineSensing()
    : marker_(), line_(), periodElapsedSemphr_(xSemaphoreCreateBinaryStatic(&periodElapsedSemphrBuffer_)) {}

/* 初期化 */
bool LineSensing::Initialize() {
  /* 赤外線LED立ち上がり待ちタイマー初期化 */
  if (HAL_TIM_RegisterCallback(&htim7, HAL_TIM_PERIOD_ELAPSED_CB_ID, PeriodElapsedCallback) != HAL_OK) {
    return false;
  }
  /* マーカーセンサー初期化 */
  if (!MarkerAdc::Instance().Initialize()) {
    return false;
  }
  /* ラインセンサー初期化 */
  if (!LineAdc::Instance().Initialize()) {
    return false;
  }
  /* タスク作成 */
  if (!TaskCreate("LineSensing", configMINIMAL_STACK_SIZE, kPriorityLineSensing)) {
    return false;
  }
  return true;
}

/* 不揮発メモリからキャリブレーション情報を復元 */
bool LineSensing::LoadCalibrationData() {
  std::array<uint16_t, LineImpl::kNum> lineMin;
  std::array<uint16_t, LineImpl::kNum> lineMax;
  std::array<float, LineImpl::kNum> lineCoeff;
  std::array<uint16_t, MarkerImpl::kNum> markerMax;
  /* 補正値を読み出し */
  if (!NonVolatileData::ReadLineSensorCalibrationData(lineMin, lineMax, lineCoeff, markerMax)) {
    return false;
  }
  printf(" ----- NonVolatileData::ReadLineSensorCalibrationData ----- \r\n");
  for (uint32_t ch = 0; ch < 8; ch++) {
    printf("Right%ld Min: %d, Max: %d, Coeff: %f\r\n", ch, lineMin[ch], lineMax[ch],
           static_cast<double>(lineCoeff[ch]));
    printf("Left %ld Min: %d, Max: %d, Coeff: %f\r\n", ch, lineMin[ch + 8], lineMax[ch + 8],
           static_cast<double>(lineCoeff[ch + 8]));
  }
  printf("Marker Right Max: %d\r\n", markerMax[0]);
  printf("Marker Left  Max: %d\r\n", markerMax[1]);
  marker_.SetCalibration(markerMax);
  line_.SetCalibration(lineMin, lineMax, lineCoeff);
  return true;
}

/* キャリブレーション */
bool LineSensing::StoreCalibrationData(uint32_t sampleNum) {
  std::array<uint16_t, LineImpl::kNum> lineMin;
  std::array<uint16_t, LineImpl::kNum> lineMax;
  std::array<float, LineImpl::kNum> lineCoeff;
  std::array<uint16_t, MarkerImpl::kNum> markerMax;
  auto &markerAdc = MarkerAdc::Instance();
  auto &lineAdc = LineAdc::Instance();
  bool failed = false;
  for (uint32_t n = 0; n < sampleNum; n++) {
    if (!Periodic::WaitPeriodicNotify()) {
      return false;
    }
    if (!TurnOnIrLed() || !lineAdc.Fetch() || !markerAdc.Fetch()) {
      failed = true;
      break;
    }
    /* 最大値・最小値を取得 */
    for (uint32_t num = 0; num < lineAdc.kNum; num++) {
      uint16_t raw = lineAdc.GetRaw(num);
      lineMax[num] = std::max(lineMax[num], raw);
      lineMin[num] = std::min(lineMin[num], raw);
    }
    for (uint32_t num = 0; num < markerAdc.kNum; num++) {
      uint16_t raw = markerAdc.GetRaw(num);
      markerMax[num] = std::max(markerMax[num], raw);
    }
    TurnOffIrLed();
  }
  if (failed) {
    TurnOffIrLed();
    return false;
  }
  /* 係数を計算 */
  for (uint32_t num = 0; num < lineAdc.kNum; num++) {
    lineCoeff[num] = 1 / static_cast<float>(lineMax[num] - lineMin[num]);
  }
  printf(" ----- LineSensing::StoreCalibrationData(%ld) ----- \r\n", sampleNum);
  for (uint32_t ch = 0; ch < 8; ch++) {
    printf("Right%ld Min: %d, Max: %d, Coeff: %f\r\n", ch, lineMin[ch], lineMax[ch],
           static_cast<double>(lineCoeff[ch]));
    printf("Left %ld Min: %d, Max: %d, Coeff: %f\r\n", ch, lineMin[ch + 8], lineMax[ch + 8],
           static_cast<double>(lineCoeff[ch + 8]));
  }
  printf("Marker Right Max: %d\r\n", markerMax[0]);
  printf("Marker Left  Max: %d\r\n", markerMax[1]);
  if (!NonVolatileData::WriteLineSensorCalibrationData(lineMin, lineMax, lineCoeff, markerMax)) {
    return false;
  }
  marker_.SetCalibration(markerMax);
  line_.SetCalibration(lineMin, lineMax, lineCoeff);
  return true;
}

/* タスク */
void LineSensing::TaskEntry() {
  uint32_t notify = 0;
  auto &odometry = MotionSensing::MotionSensing::Instance().Odometry();
  Periodic::Instance().Add(TaskHandle());
  while (true) {
    TaskNotifyWaitStart();
    printf("%ld: LineSensing start\r\n", HAL_GetTick());
    line_.Reset();
    marker_.Reset();
    while (true) {
      /* TODO: タイムアウト */
      if (!TaskNotifyWait(notify)) {
        /* TODO: エラーハンドリング */
      }
      if (notify & kTaskNotifyBitStop) {
        break;
      }
      if (notify & kTaskNotifyBitPeriodic) {
        /* 交差・マーカー補正によって距離が補正されると困るため、確実に補正される前の値を使用すること */
        TurnOnIrLed();
        float distance = odometry.GetDisplacement().trans;
        line_.Update(distance);
        if (line_.IsCrossPassed()) {
          marker_.SetIgnore(distance);
        }
        marker_.Update(distance);
        TurnOffIrLed();
      }
    }
  }
}
}  // namespace LineSensing
