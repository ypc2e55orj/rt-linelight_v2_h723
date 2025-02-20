#include "MotionPlaning/PositionCorrector.h"

/* C++ */
#include <cmath>

/* Project */
#include "Config.h"

namespace MotionPlaning {

/* コンストラクタ */
PositionCorrector::PositionCorrector() { ResetStored(); }

/* 記録したランドマークをリセット */
void PositionCorrector::ResetStored() {
  curvatureVec_.clear();
  crossLineVec_.clear();
}

/* 参照インデックスをリセット */
void PositionCorrector::ResetIndex() {
  curvatureIndex_ = 0;
  crossLineIndex_ = 0;
}

/* ランドマークを更新 */
void PositionCorrector::Store(Landmark mark, float distance) {
  switch (mark) {
    case kCurvatureMark:
      curvatureVec_.push_back(distance);
      break;
    case kCrossLine:
      crossLineVec_.push_back(distance);
      break;
    default:
      break;
  }
}

/* 位置補正値を取得 */
float PositionCorrector::Correct(Landmark mark, /* 補正に使用するランドマークの種類 */
                                 float distance /* 補正前の距離 */
) {
  float correctDist = 0.0f;
  switch (mark) {
    case kCurvatureMark: {
      /* 曲率マーカー */
      bool ret = SearchNearest(curvatureVec_, distance, kCorrectionAllowErrorCurvature, curvatureIndex_, correctDist);
      return ret ? correctDist : distance;
    }
    case kCrossLine: {
      /* 交差 */
      bool ret = SearchNearest(crossLineVec_, distance, kCorrectionAllowErrorCrossLine, crossLineIndex_, correctDist);
      return ret ? correctDist : distance;
    }
    default:
      return distance;
  }
}

/* 記録したランドマークを取得 */
const std::vector<float> &PositionCorrector::Get(Landmark mark) const {
  switch (mark) {
    case kCurvatureMark:
      return curvatureVec_;
    case kCrossLine:
      return crossLineVec_;
    default:
      return curvatureVec_;
  }
}

/* 一番近い補正位置を取得 */
bool PositionCorrector::SearchNearest(const std::vector<float> &v, /* 補正位置のベクタ */
                                      float distance,              /* 補正位置 */
                                      float allowError,            /* 補正を許容するずれ */
                                      uint32_t &minIndex,          /* 検索下限インデックス */
                                      float &nearest               /* 結果 */
) {
  for (uint32_t i = minIndex; i < v.size(); i++) {
    if (std::abs(v[i] - distance) < allowError) {
      minIndex = i;
      nearest = v[i];
      return true;
    }
  }
  return false;
}
}  // namespace MotionPlaning
