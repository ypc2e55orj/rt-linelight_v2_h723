#ifndef MOTIONSENSING_ODOMETRY_H_
#define MOTIONSENSING_ODOMETRY_H_

/* Project */
#include "Config.h"
#include "Data/MovingAverage.h"
#include "Wrapper/Mutex.h"

namespace MotionSensing {
struct Polar {
  float trans, rot;
};
struct Pose {
  float x, y, theta;
};

class Odometry {
 public:
  /* コンストラクタ */
  Odometry();

  /* リセット */
  void Reset();

  /* オドメトリ・デッドレコニングを更新 */
  void Update(float wheelAngleRight, /* エンコーダーから取得した車輪角度(右) [rad] */
              float wheelAngleLeft,  /* エンコーダーから取得した車輪角度(左) [rad] */
              float accelY,          /* IMUから取得したy軸加速度 [m/ss] */
              float yawRate          /* IMUから取得したz軸角速度 [rad/s] */
  );

  /* 周期での変位距離を取得 */
  float GetDisplacementTranslateDelta() const;

  /* 加速度を取得 */
  Polar GetAcceleration() const;
  /* 速度を取得 */
  Polar GetVelocity() const;
  /* 変位を取得 */
  Polar GetDisplacement() const;
  /* 姿勢を取得 */
  Pose GetPose() const;

 private:
  mutable Mutex mtx_;
  float deltaDispTrans_;

  MovingAverage<float, float, kEncoderNumMovingAverage> transVeloAvg_;

  Polar acc_{}; /* 加速度 [m/ss] */
  Polar vel_{}; /* 速度 [m/s]*/
  Polar dis_{}; /* 位置 [m] */
  Pose pose_{}; /* 姿勢 */
};
}  // namespace MotionSensing

#endif  // MOTIONSENSING_ODOMETRY_H_
