#ifndef MOTIONSENSING_MOTIONSENSING_H_
#define MOTIONSENSING_MOTIONSENSING_H_

/* Projects */
#include "Odometry.h"
#include "Wrapper/Task.h"

namespace MotionSensing {
class MotionSensing final : public Task<MotionSensing> {
 public:
  /* コンストラクタ */
  MotionSensing();

  /* 初期化 */
  bool Initialize();

  /* キャリブレーション */
  bool CalibrateImu(int32_t sampleNum);

  /* オドメトリを取得 */
  Odometry &GetOdometry() { return odometry_; }

 protected:
  /* タスク */
  void TaskEntry() final;

 private:
  Odometry odometry_;
};
}  // namespace MotionSensing
#endif  // MOTIONSENSING_MOTIONSENSING_H_
