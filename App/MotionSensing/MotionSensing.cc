#include "MotionSensing/MotionSensing.h"

/* Projects */
#include "Config.h"
#include "MotionSensing/Encoder.h"
#include "MotionSensing/Imu.h"
#include "Periodic.h"

/* C++ */
#include <cstdio>

namespace MotionSensing {
MotionSensing::MotionSensing() : odometry_() {}

/* 初期化 */
bool MotionSensing::Initialize() {
  /* エンコーダー初期化 */
  if (!Encoder::Instance().Initialize()) {
    return false;
  }
  /* IMU初期化 */
  if (!Imu::Instance().Initialize()) {
    return false;
  }
  /* タスク作成 */
  if (!TaskCreate("MotionSensing", configMINIMAL_STACK_SIZE, kPriorityMotionSensing)) {
    return false;
  }
  return true;
}

/* キャリブレーション */
bool MotionSensing::CalibrateImu(int32_t sampleNum) {
  Imu::Offset offset{};
  std::array<int32_t, 6> acc{};
  auto &imu = Imu::Instance();
  imu.Reset();
  imu.SetOffset(offset);
  /* サンプル数の半分ほどのダミー取得 */
  for (int32_t n = 0; n < sampleNum / 2; n++) {
    if (!Periodic::WaitPeriodicNotify()) {
      return false;
    }
    if (!imu.Update()) {
      imu.Reset();
      return false;
    }
  }
  /* キャリブレーション */
  for (int32_t n = 0; n < sampleNum; n++) {
    if (!Periodic::WaitPeriodicNotify()) {
      return false;
    }
    if (!imu.Update()) {
      imu.Reset();
      return false;
    }
    auto gyro = imu.GetGyroRaw();
    auto accel = imu.GetAccelRaw();

    for (int i = 0; i < 3; i++) {
      acc[i] += gyro[i];
      acc[i + 3] += accel[i];
    }
  }
  /* TODO:
   * 補正値をフラッシュに保存しておいてフラッシュから復元するメソッドを用意 */
  printf(" ----- MotionSensing::CalibrateImu(%ld) ----- \r\n", sampleNum);
  for (int32_t n = 0; n < 6; n++) {
    offset[n] = static_cast<int16_t>(acc[n] / sampleNum);
    printf("offset[%ld]: %d\r\n", n, offset[n]);
  }
  imu.SetOffset(offset);

  return true;
}

/* タスク */
void MotionSensing::TaskEntry() {
  uint32_t notify = 0;
  Periodic::Instance().Add(TaskHandle());
  while (true) {
    TaskNotifyWaitStart();
    Imu::Instance().Reset();
    Encoder::Instance().Reset();
    odometry_.Reset();
    while (true) {
      /* TODO: タイムアウト */
      if (!TaskNotifyWait(notify)) {
        /* TODO: エラーハンドリング */
      }
      if (notify & kTaskNotifyBitStop) {
        break;
      }
      if (notify & kTaskNotifyBitPeriodic) {
        auto &imu = Imu::Instance();
        auto &encoder = Encoder::Instance();
        imu.Update();
        encoder.Update();
        auto angleDiff = encoder.GetDiff();
        odometry_.Update(angleDiff[0], angleDiff[1], imu.GetAccelY(), imu.GetYawRate());
      }
    }
  }
}
}  // namespace MotionSensing
