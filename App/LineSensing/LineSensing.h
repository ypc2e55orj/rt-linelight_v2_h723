#ifndef LINESENSING_LINESENSING_H_
#define LINESENSING_LINESENSING_H_

/* Projects */
#include "LineMarker.h"
#include "Wrapper/Task.h"

namespace LineSensing {
class LineSensing final : public Task<LineSensing> {
 public:
  /* コンストラクタ */
  LineSensing();

  /* 初期化 */
  bool Initialize();

  /* キャリブレーション */
  bool Calibrate(uint32_t sampleNum);

  /* ラインを取得 */
  const Line &GetLine() { return line_; }

  /* マーカーを取得 */
  const Marker &GetMarker() { return marker_; }

 protected:
  /* タスク */
  void TaskEntry() final;

 private:
  Marker marker_;
  Line line_;
};
}  // namespace LineSensing

#endif  // LINESENSING_LINESENSING_H_
