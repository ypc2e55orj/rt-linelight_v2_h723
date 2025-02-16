#include "MotionSensing/Odometry.h"

/* C++ */
#include <cmath>
#include <mutex>

/* Project */
#include "Config.h"

namespace MotionSensing {
/* コンストラクタ */
OdometryImpl::OdometryImpl() { Reset(); }

/* リセット */
void OdometryImpl::Reset() {
  std::scoped_lock<Mutex> lock(mtx_);
  deltaDispTrans_ = 0.0f;
  acc_ = {}, vel_ = {}, dis_ = {}, pose_ = {};
  transVeloAvg_.Reset();
}

/* オドメトリ・デッドレコニングを更新 */
void OdometryImpl::Update(float wheelDeltaAngleRight, /* エンコーダーから取得した車輪変化角度(右) [rad] */
                      float wheelDeltaAngleLeft, /* エンコーダーから取得した車輪変化角度(左) [rad] */
                      float accelY,              /* IMUから取得したy軸加速度 [m/ss] */
                      float yawRate              /* IMUから取得したz軸角速度 [rad/s] */
) {
  std::scoped_lock<Mutex> lock(mtx_);
  deltaDispTrans_ = (wheelDeltaAngleRight + wheelDeltaAngleLeft) * kWheelRadius / 2.0f;
  transVeloAvg_.Update(deltaDispTrans_ / kPeriodicNotifyInterval);

  acc_.trans = accelY;
  acc_.rot = (yawRate - vel_.rot) / kPeriodicNotifyInterval;
  vel_.trans = transVeloAvg_.Get();
  vel_.rot = yawRate;
  dis_.trans += deltaDispTrans_;
  dis_.rot += yawRate * kPeriodicNotifyInterval;

  pose_.theta = dis_.rot;
  pose_.x += (vel_.trans * kPeriodicNotifyInterval) * cosf(pose_.theta);
  pose_.y += (vel_.trans * kPeriodicNotifyInterval) * sinf(pose_.theta);
}

/* 周期での変位距離を取得 */
float OdometryImpl::GetDisplacementTranslateDelta() const { return deltaDispTrans_; }

/* 加速度を取得 */
Polar OdometryImpl::GetAcceleration() const {
  std::scoped_lock<Mutex> lock(mtx_);
  return acc_;
}

/* 速度を取得 */
Polar OdometryImpl::GetVelocity() const {
  std::scoped_lock<Mutex> lock(mtx_);
  return vel_;
}

/* 変位を取得 */
Polar OdometryImpl::GetDisplacement() const {
  std::scoped_lock<Mutex> lock(mtx_);
  return dis_;
}

/* 姿勢を取得 */
Pose OdometryImpl::GetPose() const {
  std::scoped_lock<Mutex> lock(mtx_);
  return pose_;
}
}  // namespace MotionSensing
