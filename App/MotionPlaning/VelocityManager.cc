#include "MotionPlaning/VelocityManager.h"

/* Project */
#include "Config.h"

/* C++ */
#include <algorithm>
#include <cstdio>

namespace MotionPlaning {
/* コンストラクタ */
RadiusExplorer::RadiusExplorer() { Reset(); }

/* リセット */
void RadiusExplorer::Reset() {
  accDeltaDistance_ = 0.0f;
  accYawRate_ = 0.0f;
  exploringVec_.clear();
  explored_ = false;
}

/* 探索更新 */
void RadiusExplorer::Update(float deltaDistance, /* 周期での変位距離 [m] */
                            float yawRate        /* 角速度 [rad/s] */
) {
  accDeltaDistance_ += deltaDistance;
  accYawRate_ += yawRate * kPeriodicNotifyInterval;
  if (accDeltaDistance_ >= kMappingDistance) {
    exploringVec_.push_back({accDeltaDistance_, accYawRate_});
    accDeltaDistance_ = 0.0f;
    accYawRate_ = 0.0f;
  }
}

/* 探索停止 */
void RadiusExplorer::Explored() { explored_ = true; }

/* 探索済みか */
bool RadiusExplorer::IsExplored() const { return explored_; }

/* ログ取得 */
const std::vector<ExploredLog> &RadiusExplorer::Get() const { return exploringVec_; }

/*  */

/* コンストラクタ */
VelocityMapGenerator::VelocityMapGenerator(const RadiusExplorer &radiusExplorer) : radiusExplorer_(radiusExplorer) {
  ResetIndex();
  ResetGenerated();
}

/* 計算 */
bool VelocityMapGenerator::Generate(const std::vector<RadiusVelocityLimit> &lv, /* 半径での上限速度 [m/s] */
                                    float startVelo,                            /* 開始速度 [m/s] */
                                    float accel,                                /* 加速度 [m/ss] */
                                    float decel,                                /* 減速度 [m/ss] */
                                    uint32_t shift                              /* 速度テーブルをずらす */
) {
  auto &vv = velocityVec_;
  if (!radiusExplorer_.IsExplored()) {
    return false;
  }

  vv.clear();

  /* 開始速度 */
  vv.push_back(startVelo);
  /* 上限速度マップを作成 */
  const auto &ev = radiusExplorer_.Get();
  for (uint32_t i = 1; i < ev.size(); i++) {
    auto theta = std::max(std::abs(ev[i].yaw), kMappingMinAngle);
    auto radius = std::min(ev[i].distance / theta, kMappingMaxRadius);
    uint32_t li = 0;
    for (; li < lv.size() - 1; li++) {
      if (radius < lv[li].minRadius) {
        break;
      }
    }
    vv.push_back(lv[li].maxVelocity);
  }
  /* 実現可能な速度に修正 */
  /* 減速 */
  for (uint32_t i = vv.size() - 1; i >= 1; i--) {
    auto diff = vv[i - i] - vv[i];
    auto dist = ev[i].distance;
    if (diff > 0.0f) {
      auto t = dist / diff;
      auto a = diff / t;
      if (a > decel) {
        vv[i - 1] = vv[i] + decel * dist;
      }
    }
  }
  /* 加速 */
  for (uint32_t i = 0; i < vv.size() - 1; i++) {
    auto diff = vv[i + 1] - vv[i];
    auto dist = ev[i].distance;
    if (diff > 0.0f) {
      auto t = dist / diff;
      auto a = diff / t;
      if (a > accel) {
        vv[i + 1] = vv[i] + accel * dist;
      }
    }
  }
  /* 速度テーブルをずらす */
  {
    auto lastVelo = vv.back();
    std::shift_left(vv.begin(), vv.end(), shift);
    for (uint32_t s = 0; s < shift; s++) {
      vv.push_back(lastVelo);
    }
  }

  fputc(2, stdout);
  for (uint32_t i = 0; i < vv.size(); i++) {
    fprintf(stdout, "%f\n", static_cast<double>(vv[i]));
  }
  fputc(3, stdout);
  fflush(stdout);

  calculated_ = true;

  return true;
}

/* 計算済みか */
bool VelocityMapGenerator::IsGenerated() const { return calculated_; }

/* 計算済みテーブルをリセット */
void VelocityMapGenerator::ResetGenerated() { calculated_ = false; }

/* 更新 */
void VelocityMapGenerator::UpdateIndex() {
  auto &elv = radiusExplorer_.Get();
  if (accDeltaDistance_ >= logAccDistance_) {
    logAccDistance_ += elv[index_].distance;
    if (++index_ >= elv.size()) {
      index_ = elv.size() - 1;
    }
  }
}

/* 位置を更新 */
void VelocityMapGenerator::UpdateDistance(float deltaDistance) { accDeltaDistance_ += deltaDistance; }

/* 位置を取得 */
float VelocityMapGenerator::GetTotalDistance() const { return accDeltaDistance_; }

/* 補正位置を設定 */
void VelocityMapGenerator::CorrectDistance(float distance) { accDeltaDistance_ = distance; }

/* リセット */
void VelocityMapGenerator::ResetIndex() {
  index_ = 0;
  logAccDistance_ = 0.0f;
  accDeltaDistance_ = 0.0f;
}

/* 添字を取得 */
uint32_t VelocityMapGenerator::GetIndex() const { return index_; }

/* 現在の目標速度を取得 */
float VelocityMapGenerator::GetVelocity() const { return velocityVec_[index_]; }

/* 次の目標速度を取得 */
float VelocityMapGenerator::GetNextVelocity() const { return velocityVec_[index_ + 1]; }

/* 生成したテーブルを取得 */
const std::vector<float> &VelocityMapGenerator::Get() const { return velocityVec_; }
}  // namespace MotionPlaning
