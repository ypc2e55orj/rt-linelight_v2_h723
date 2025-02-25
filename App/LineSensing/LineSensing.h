#ifndef LINESENSING_LINESENSING_H_
#define LINESENSING_LINESENSING_H_

/* STM32CubeMX */
#include <main.h>

/* Projects */
#include "Line.h"
#include "Marker.h"
#include "Wrapper/Task.h"

namespace LineSensing {
class LineSensing final : public Task<LineSensing> {
 public:
  /* コンストラクタ */
  LineSensing();

  /* 初期化 */
  bool Initialize();

  /* 不揮発メモリからキャリブレーション情報を復元 */
  bool LoadCalibrationData();

  /* キャリブレーション */
  bool StoreCalibrationData(uint32_t sampleNum);

  /* ラインを取得 */
  const LineImpl &Line() { return line_; }

  /* マーカーを取得 */
  const MarkerImpl &Marker() { return marker_; }

 protected:
  /* タスク */
  void TaskEntry() final;

 private:
  MarkerImpl marker_;
  LineImpl line_;

  /* IR LED点灯待ち */
  static void PeriodElapsedCallback(TIM_HandleTypeDef *);
  StaticSemaphore_t periodElapsedSemphrBuffer_;
  SemaphoreHandle_t periodElapsedSemphr_; /* 立ち上がり待ちセマフォ */

  bool TurnOnIrLed();
  void TurnOffIrLed();
};
}  // namespace LineSensing

#endif  // LINESENSING_LINESENSING_H_
