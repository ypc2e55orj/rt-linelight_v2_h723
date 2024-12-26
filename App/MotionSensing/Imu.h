#ifndef MOTIONSENSING_IMU_H_
#define MOTION_SENSING_IMU_H_

/* C++ */
#include <array>

/* Project */
#include "Data/Singleton.h"
#include "Wrapper/Mutex.h"

namespace MotionSensing {
/**
 * MARK: Imu
 */
class Imu final : public Singleton<Imu> {
 public:
  using Offset = std::array<int16_t, 6>;
  using Raw = std::array<int16_t, 3>;

  /* 初期化 */
  bool Initialize();

  /* 内部値を更新(1ms周期で呼び出すこと) */
  bool Update();

  /* リセット */
  void Reset();

  /* オフセットを取得 */
  void GetOffset(Offset &offset);

  /* オフセットを設定 */
  void SetOffset(const Offset &offset);

  /* x軸加速度を取得 [m/ss] */
  float GetAccelX();

  /* y軸加速度を取得 [m/ss] */
  float GetAccelY();

  /* ヨーレートを取得 [rad/s] */
  float GetYawRate();

  /* 加速度センサの生値を取得 */
  Raw GetAccelRaw();

  /* ジャイロセンサの生値を取得 */
  Raw GetGyroRaw();

 private:
  Mutex mtx_;

  Offset offset_{};
  Raw gyro_{};
  Raw accel_{};
};
}  // namespace MotionSensing
#endif  // MOTIONSENSING_IMU_H_
