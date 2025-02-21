#ifndef MOTIONPLANING_VELOCITY_MAPPING_H_
#define MOTIONPLANING_VELOCITY_MAPPING_H_

/* Project */
#include "Config.h"
#include "Wrapper/New.h"

/* C++ */
#include <array>
#include <cstdint>
#include <vector>

namespace MotionPlaning {
/* 加減速探索 */
class VelocityMapping {
 public:
  enum class CorrectType {
    kCurveMarker,
    kCrossLine,
  };

  /* コンストラクタ */
  VelocityMapping();

  /* 探索リセット */
  void ResetSearchRunning();
  /* 探索更新 */
  void UpdateSearchRunningCurvePoint(float deltaDistance, /* 制御周期での変化距離 [m] */
                                     float yawRate        /* 角速度 [rad/s] */
  );
  void AddSearchRunningCorrectPoint(CorrectType type, float distance);
  /* 探索成功*/
  void SuccessSearchRunning();
  /* 探索が成功したか */
  bool IsSearched();
  /* 探索データを取得 */
  uint16_t GetSearchRunningNumPoints();
  const std::array<float, kMappingMaxPoints> &GetDeltaDistanceArray();
  const std::array<float, kMappingMaxPoints> &GetDeltaAngleArray();
  const std::array<float, kCorrectionMaxPoints> &GetCrossLinePoints(uint16_t &num);
  const std::array<float, kCorrectionMaxPoints> &GetCurveMarkerPoints(uint16_t &num);

  /* 不揮発メモリから読み出し */
  bool LoadSearchRunningPoints();
  /* 不揮発メモリに書き込み */
  bool StoreSearchRunningPoints();

  /* 速度テーブルを計算 */
  bool CalculatVelocityTable(const std::vector<float> &minRadiusVec,   /* 速度を変更する半径[m] */
                             const std::vector<float> &maxVelocityVec, /* 半径における並進速度[m/s] */
                             float startVelo,                          /* 開始速度 [m/s] */
                             float accel,                              /* 加速度 [m/ss] */
                             float decel                               /* 減速度 [m/ss] */
  );
  /* 速度テーブルをリセット */
  void ResetVelocityTable();
  /* 速度テーブルがあるか */
  bool HasVelocityTable();
  /* 速度テーブルを取得 */
  const std::vector<float> &GetVelocityTable();

  /* 最短走行リセット */
  void ResetFastRunning();
  /* 最短走行更新 */
  void UpdateFastRunning(float deltaDistance,                 /* 制御周期での変化距離 [m] */
                         bool isCrossLine, bool isCurveMarker /* 補正位置があるか */
  );
  /* 速度を取得 */
  void GetFastRunningVelocity(float &now, float &next);
  /* 走行位置を取得 */
  float GetFastRunningDistance();
  /* 参照している速度テーブルのインデックスを取得 */
  uint16_t GetFastRunningPoint();

 private:
  /* 探索 */
  float searchAccDistance_;                                   /*  記録中の距離 [m] */
  float searchAccYawRate_;                                    /*  記録中の変化角度 [rad] */
  uint16_t numSearchRunningPoints_;                           /*  記録点数 */
  std::array<float, kMappingMaxPoints> deltaDistanceArray_;   /*  距離 [m] */
  std::array<float, kMappingMaxPoints> deltaAngleArray_;      /*  距離での変化角度 [rad] */
  uint16_t numCrossLinePoints_;                               /* 交差点の位置数 */
  std::array<float, kCorrectionMaxPoints> crossLinePoints_;   /* 交差点の位置 */
  uint16_t numCurveMarkerPoints_;                             /* マーカーの位置数 */
  std::array<float, kCorrectionMaxPoints> curveMarkerPoints_; /* マーカーの位置 */
  bool searched_;

  /* 速度テーブル */
  std::vector<float> velocityVec_; /* 速度テーブル [m/s] */
  bool hasVelocityTable_;          /* 速度テーブルがあるか */

  /* 最短 */
  uint16_t fastRunningPoint_;        /* 速度テーブル索引インデックス */
  float fastAccDistance_;            /* 最短総移動距離 [m] */
  float fastVelocityChangeDistance_; /* 次の速度変化距離 [m] */

  uint16_t fastCrossLinePoint_;   /* 交差点の補正位置 */
  uint16_t fastCurveMarkerPoint_; /* マーカーの補正位置 */
};

}  // namespace MotionPlaning

#endif  // MOTIONPLANING_VELOCITY_MAPPING_H_
