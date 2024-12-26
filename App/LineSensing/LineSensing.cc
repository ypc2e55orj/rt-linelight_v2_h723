#include "LineSensing/LineSensing.h"

/* Projects */
#include "Config.h"
#include "LineSensing/Calibrator.h"
#include "LineSensing/LineMarkerAdc.h"
#include "MotionSensing/MotionSensing.h"
#include "Periodic.h"

namespace LineSensing {
LineSensing::LineSensing() : marker_(), line_() {}

/* 初期化 */
bool LineSensing::Initialize() {
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

/* キャリブレーション */
bool LineSensing::Calibrate(uint32_t sampleNum) {
  Calibrator<MarkerAdc> markerCalibrator(MarkerAdc::Instance());
  Calibrator<LineAdc> lineCalibrator(LineAdc::Instance());
  SetIr(true);
  vTaskDelay(pdMS_TO_TICKS(5));
  for (uint32_t n = 0; n < sampleNum; n++) {
    if (!Periodic::WaitPeriodicNotify()) {
      return false;
    }
    if (!markerCalibrator.Fetch()) {
      return false;
    }
    if (!lineCalibrator.Fetch()) {
      return false;
    }
  }
  /* TODO:
   * 補正値をフラッシュに保存しておいてフラッシュから復元するメソッドを用意 */
  markerCalibrator.Calculate();
  lineCalibrator.Calculate();
  auto markerMax = markerCalibrator.GetMax();
  auto lineMax = lineCalibrator.GetMax();
  auto lineOffset = lineCalibrator.GetOffset();
  printf(" ----- LineSensing::Calibrate(%ld) ----- \r\n", sampleNum);
  for (uint32_t ch = 0; ch < 8; ch++) {
    printf("Right%ld Max: %d, Offset: %f\r\n", ch, lineMax[ch], static_cast<double>(lineOffset[ch]));
    printf("Left %ld Max: %d, Offset: %f\r\n", ch, lineMax[ch + 8], static_cast<double>(lineOffset[ch + 8]));
  }
  printf("Marker Right Max: %d\r\n", markerMax[0]);
  printf("Marker Left  Max: %d\r\n", markerMax[1]);
  marker_.SetThreshold(markerMax);
  line_.SetThreshold(lineMax);
  line_.SetOffset(lineOffset);
  SetIr(false);
  return true;
}

/* タスク */
void LineSensing::TaskEntry() {
  uint32_t notify = 0;
  auto &odometry = MotionSensing::MotionSensing::Instance().GetOdometry();
  Periodic::Instance().Add(TaskHandle());
  while (true) {
    TaskNotifyWaitStart();
    printf("%ld: LineSensing start\r\n", HAL_GetTick());
    SetIr(true);
    line_.Reset();
    marker_.Reset();
    while (true) {
      /* TODO: タイムアウト */
      if (!TaskNotifyWait(notify)) {
        /* TODO: エラーハンドリング */
      }
      if (notify & kTaskNotifyBitStop) {
        SetIr(false);
        break;
      }
      if (notify & kTaskNotifyBitPeriodic) {
        /* 交差・マーカー補正によって距離が補正されると困るため、確実に補正される前の値を使用すること */
        float distance = odometry.GetDisplacement().trans;
        line_.Update(distance);
        if (line_.IsCrossPassed()) {
          marker_.SetIgnore(distance);
        }
        marker_.Update(distance);
      }
    }
  }
}
}  // namespace LineSensing
