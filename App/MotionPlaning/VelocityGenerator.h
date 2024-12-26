#ifndef MOTIONPLANING_VELOCITYGENERATOR_H_
#define MOTIONPLANING_VELOCITYGENERATOR_H_

/* C++ */
#include <cstdint>

class SlopeVelocityGenerator {
 public:
  struct Profile {
    float startVelocity; /* 開始速度 [m/s] */
    float maxVelocity;   /* 最大速度 [m/s] */
    float endVelocity;   /* 終了速度 [m/s] */
    float acceleration;  /* 加速度 [m/s^2] */
    float deceleration;  /* 減速度 [m/s^2] */
    float distance;      /* 移動距離 [m] */
  };

  /* 計算 */
  void Generate(const Profile &profile);
  /* 時刻tにおける速度を取得 */
  float GetVelocity(uint32_t ms);

  /* 加速時間を取得 */
  uint32_t GetAccelerationTime();
  /* 加速距離を取得 */
  float GetAccelerationDistance();
  /* 等速時間を取得 */
  uint32_t GetConstantSpeedTime();
  /* 等速距離を取得 */
  float GetConstantSpeedDistance();
  /* 減速時間を取得 */
  uint32_t GetDecelerationTime();
  /* 減速距離を取得 */
  float GetDecelerationDistance();
  /* 走行時間を取得 */
  uint32_t GetTotalTime();
  /* 走行距離を取得 */
  float GetTotalDistance();

 private:
  float t0_, t1_, t2_;
  float x0_, x1_, x2_;
  Profile profile_;
};

#endif  // MOTIONPLANING_VELOCITYGENERATOR_H_
