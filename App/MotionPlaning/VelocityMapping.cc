#include "MotionPlaning/VelocityMapping.h"

/* Project */
#include "Config.h"
#include "NonVolatileData.h"

/* C++ */
#include <algorithm>
#include <cstdio>

namespace MotionPlaning {
/* コンストラクタ */
VelocityMapping::VelocityMapping() {
  ResetSearchRunning();
  ResetFastRunning();
  ResetVelocityTable();
}

/* 探索リセット */
void VelocityMapping::ResetSearchRunning() {
  searchAccDistance_ = 0.0f;
  searchAccYawRate_ = 0.0f;
  numSearchRunningPoints_ = 0;
  deltaDistanceArray_.fill(0.0f);
  deltaAngleArray_.fill(0.0f);
  numCrossLinePoints_ = 0;
  crossLinePoints_.fill(0.0f);
  numCurveMarkerPoints_ = 0;
  curveMarkerPoints_.fill(0.0f);
  searched_ = false;
}
/* 探索更新 */
void VelocityMapping::UpdateSearchRunningCurvePoint(float deltaDistance, /* 変位距離 [m] */
                                                    float yawRate        /* 角速度 [rad/s] */
) {
  searchAccDistance_ += deltaDistance;
  searchAccYawRate_ += yawRate * kPeriodicNotifyInterval;
  if (searchAccDistance_ >= kMappingDistance) {
    if (numSearchRunningPoints_ < kMappingMaxPoints) {
      deltaDistanceArray_[numSearchRunningPoints_] = searchAccDistance_;
      deltaAngleArray_[numSearchRunningPoints_] = searchAccYawRate_;
      numSearchRunningPoints_++;
    }
    searchAccDistance_ = 0.0f;
    searchAccYawRate_ = 0.0f;
  }
}
void VelocityMapping::AddSearchRunningCorrectPoint(CorrectType type, float distance) {
  switch (type) {
    case CorrectType::kCurveMarker:
      if (numCurveMarkerPoints_ < kCorrectionMaxPoints) {
        curveMarkerPoints_[numCurveMarkerPoints_] = distance;
        numCurveMarkerPoints_++;
      }
      break;
    case CorrectType::kCrossLine:
      if (numCrossLinePoints_ < kCorrectionMaxPoints) {
        crossLinePoints_[numCrossLinePoints_] = distance;
        numCrossLinePoints_++;
      }
      break;
  }
}
/* 探索成功*/
void VelocityMapping::SuccessSearchRunning() { searched_ = true; }
/* 探索が成功したか */
bool VelocityMapping::IsSearched() { return searched_; }
/* 探索データを取得 */
uint16_t VelocityMapping::GetSearchRunningNumPoints() { return numSearchRunningPoints_; }
const std::array<float, kMappingMaxPoints> &VelocityMapping::GetDeltaDistanceArray() { return deltaDistanceArray_; }
const std::array<float, kMappingMaxPoints> &VelocityMapping::GetDeltaAngleArray() { return deltaAngleArray_; }
const std::array<float, kCorrectionMaxPoints> &VelocityMapping::GetCrossLinePoints(uint16_t &num) {
  num = numCrossLinePoints_;
  return crossLinePoints_;
}
const std::array<float, kCorrectionMaxPoints> &VelocityMapping::GetCurveMarkerPoints(uint16_t &num) {
  num = numCurveMarkerPoints_;
  return curveMarkerPoints_;
}

/* 速度テーブルを計算 */
bool VelocityMapping::CalculatVelocityTable(const std::vector<float> &minRadiusVec,   /* 速度を変更する半径[m] */
                                            const std::vector<float> &maxVelocityVec, /* 半径における並進速度[m/s] */
                                            float startVelo,                          /* 開始速度 [m/s] */
                                            float accel,                              /* 加速度 [m/ss] */
                                            float decel                               /* 減速度 [m/ss] */
) {
  if (minRadiusVec.size() != minRadiusVec.size()) {
    return false;
  }

  /* 開始速度 */
  velocityVec_.push_back(startVelo);
  /* 上限速度マップを作成 */
  for (uint32_t point = 1; point < numSearchRunningPoints_; point++) {
    auto theta = std::max(std::abs(deltaAngleArray_[point]), kMappingMinAngle);
    auto radius = std::min(deltaDistanceArray_[point] / theta, kMappingMaxRadius);
    for (uint32_t i = 0; i < minRadiusVec.size(); i++) {
      if (radius <= minRadiusVec[i]) {
        velocityVec_.push_back(maxVelocityVec[i]);
        break;
      }
      if (i == minRadiusVec.size() - 1) {
        velocityVec_.push_back(maxVelocityVec.back());
      }
    }
  }
  /* 実現可能な速度に修正 */
  /* 減速 */
  for (uint32_t point_ = 0; point_ < static_cast<uint16_t>(numSearchRunningPoints_ - 1); point_++) {
    auto point = (numSearchRunningPoints_ - 1) - point_;
    if (velocityVec_[point] < velocityVec_[point - 1]) { /* now < prev */
      auto s = (std::pow(velocityVec_[point], 2) - std::pow(velocityVec_[point - 1], 2)) / (-2.0f * decel);
      if (s > deltaDistanceArray_[point]) {
        auto fix = velocityVec_[point] + deltaDistanceArray_[point] * decel;
        velocityVec_[point - 1] = std::min(fix, maxVelocityVec.back());
      }
    }
  }
  /* 加速 */
  for (uint32_t point = 0; point < static_cast<uint16_t>(numSearchRunningPoints_ - 1); point++) {
    if (velocityVec_[point] < velocityVec_[point + 1]) { /* now < next */
      auto s = (std::pow(velocityVec_[point + 1], 2) - std::pow(velocityVec_[point], 2)) / (2.0f * accel);
      if (s > deltaDistanceArray_[point + 1]) {
        auto fix = velocityVec_[point] + deltaDistanceArray_[point] * accel;
        velocityVec_[point + 1] = std::min(fix, maxVelocityVec.back());
      }
    }
  }
  hasVelocityTable_ = true;
  return true;
}
/* 速度テーブルをリセット */
void VelocityMapping::ResetVelocityTable() {
  velocityVec_.clear();
  hasVelocityTable_ = false;
}
/* 速度テーブルがあるか */
bool VelocityMapping::HasVelocityTable() { return hasVelocityTable_; }
/* 速度テーブルを取得 */
const std::vector<float> &VelocityMapping ::GetVelocityTable() { return velocityVec_; }

/* 最短走行リセット */
void VelocityMapping::ResetFastRunning() {
  fastRunningPoint_ = 0;
  fastAccDistance_ = 0.0f;
  fastVelocityChangeDistance_ = 0.0f;

  fastCrossLinePoint_ = 0;
  fastCurveMarkerPoint_ = 0;
}
/* 最短走行更新 */
void VelocityMapping::UpdateFastRunning(float deltaDistance,                 /* 制御周期での変化距離 [m] */
                                        bool isCrossLine, bool isCurveMarker /* 補正位置があるか */
) {
  /* 現在位置を積算 */
  fastAccDistance_ += deltaDistance;

  /* 位置を補正 */
  if (isCurveMarker) {
    for (; fastCurveMarkerPoint_ < numCurveMarkerPoints_; fastCurveMarkerPoint_++) {
      if (std::abs(curveMarkerPoints_[fastCurveMarkerPoint_] - fastAccDistance_) < kCorrectionAllowErrorCurvature) {
        fastAccDistance_ = curveMarkerPoints_[fastCurveMarkerPoint_];
        fastCurveMarkerPoint_++;
        break;
      }
    }
  } else if (isCrossLine) {
    for (; fastCrossLinePoint_ < numCrossLinePoints_; fastCrossLinePoint_++) {
      if (std::abs(crossLinePoints_[fastCrossLinePoint_] - fastAccDistance_) < kCorrectionAllowErrorCrossLine) {
        fastAccDistance_ = crossLinePoints_[fastCrossLinePoint_];
        fastCrossLinePoint_++;
        break;
      }
    }
  }

  /* 速度テーブルの索引位置を更新 */
  if (fastAccDistance_ >= fastVelocityChangeDistance_) {
    if (fastRunningPoint_ < numSearchRunningPoints_) {
      fastVelocityChangeDistance_ += deltaDistanceArray_[fastRunningPoint_];
      fastRunningPoint_++;
    }
  }
}
/* 速度を取得 */
void VelocityMapping::GetFastRunningVelocity(float &now, float &next) {
  now = velocityVec_[fastRunningPoint_];
  next = velocityVec_[std::min(static_cast<uint16_t>(fastRunningPoint_ + 1), numSearchRunningPoints_)];
}
/* 走行位置を取得 */
float VelocityMapping::GetFastRunningDistance() { return fastAccDistance_; }

/* 不揮発メモリから読み出し */
bool VelocityMapping::LoadSearchRunningPoints() {
  searched_ = false;
  if (!NonVolatileData::ReadVelocityMappingData(deltaDistanceArray_, deltaAngleArray_, numSearchRunningPoints_) ||
      !NonVolatileData::ReadPositionCorrectionData(crossLinePoints_, numCrossLinePoints_, curveMarkerPoints_,
                                                   numCurveMarkerPoints_)) {
    return false;
  }
  searched_ = true;
  return true;
}
/* 不揮発メモリに書き込み */
bool VelocityMapping::StoreSearchRunningPoints() {
  return NonVolatileData::WriteVelocityMappingData(deltaDistanceArray_, deltaAngleArray_, numSearchRunningPoints_) &&
         NonVolatileData::WritePositionCorrectionData(crossLinePoints_, numCrossLinePoints_, curveMarkerPoints_,
                                                      numCurveMarkerPoints_);
}
}  // namespace MotionPlaning
