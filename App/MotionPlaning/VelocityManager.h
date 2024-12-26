#ifndef MOTIONPLANING_VELOCITYMANAGER_H_
#define MOTIONPLANING_VELOCITYMANAGER_H_

/* Project */
#include "Wrapper/New.h"

/* C++ */
#include <cstdint>
#include <vector>

namespace MotionPlaning {

/* 半径探索ログ */
struct ExploredLog {
  float distance; /* 走行距離 [m] */
  float yaw;      /* 角度 [rad] */
};

/* 曲率探索クラス */
class RadiusExplorer {
 public:
  /* コンストラクタ */
  RadiusExplorer();

  /* リセット */
  void Reset();

  /* 探索更新 */
  void Update(float deltaDistance, /* 周期での変位距離 [m] */
              float yawRate        /* 角速度 [rad/s] */
  );

  /* 探索停止 */
  void Explored();

  /* 探索済みか */
  bool IsExplored() const;

  /* ログ取得 */
  const std::vector<ExploredLog> &Get() const;

 private:
  float accDeltaDistance_; /* 約0.01[m]毎にリセットされる距離 [m] */
  float accYawRate_;       /* 約0.01[m]毎にリセットされる角度 [rad] */

  std::vector<ExploredLog> exploringVec_; /* 半径 [m] */
  bool explored_;
};

/* 速度生成クラス */
class VelocityMapGenerator {
 public:
  struct RadiusVelocityLimit {
    float minRadius;   /* 下限半径 [m] */
    float maxVelocity; /* 半径最大速度 [m/s] */
  };

  /* コンストラクタ */
  explicit VelocityMapGenerator(const RadiusExplorer &radiusExplorer);

  /* 計算 */
  bool Generate(const std::vector<RadiusVelocityLimit> &rvlv, /* 半径での上限速度 [m/s] */
                float startVelo,                              /* 開始速度 [m/s] */
                float accel,                                  /* 加速度 [m/ss] */
                float decel,                                  /* 減速度 [m/ss] */
                uint32_t shift                                /* 速度テーブルをずらす */
  );

  /* 計算済みか */
  bool IsGenerated() const;

  /* 計算済みテーブルをリセット */
  void ResetGenerated();

  /* 更新 */
  void UpdateIndex();

  /* 位置を更新 */
  void UpdateDistance(float deltaDistance);

  /* 位置を取得 */
  float GetTotalDistance() const;

  /* 補正位置を設定 */
  void CorrectDistance(float distance);

  /* リセット */
  void ResetIndex();

  /* 添字を取得 */
  uint32_t GetIndex() const;

  /* 現在の目標速度を取得 */
  float GetVelocity() const;

  /* 次の目標速度を取得 */
  float GetNextVelocity() const;

  /* 生成したテーブルを取得 */
  const std::vector<float> &Get() const;

 private:
  const RadiusExplorer &radiusExplorer_;
  std::vector<float> velocityVec_; /* 速度テーブル [m/s] */
  bool calculated_;

  /* 速度索引関係 */
  float logAccDistance_; /* ログの周期距離を積算した距離 [m] */
  float accDeltaDistance_;
  uint32_t index_;
};
}  // namespace MotionPlaning

#endif  // MOTIONPLANING_VELOCITYMANAGER_H_
