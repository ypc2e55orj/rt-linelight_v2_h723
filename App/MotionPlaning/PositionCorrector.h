#ifndef MOTIONPLANING_POSITIONCORRECTOR_H_
#define MOTIONPLANING_POSITIONCORRECTOR_H_

/* Project */
#include "Wrapper/New.h"

/* C++ */
#include <cstdint>
#include <vector>

namespace MotionPlaning {
/* 位置補正用クラス */
class PositionCorrector {
 public:
  enum Landmark {
    kNone,
    kCurvatureMark,
    kCrossLine,
  };

  /* コンストラクタ */
  PositionCorrector();

  /* 記録したランドマークをリセット */
  void ResetStored();

  /* 参照インデックスをリセット */
  void ResetIndex();

  /* ランドマークを更新 */
  void Store(Landmark mark, float distance);

  /* 位置補正値を取得 */
  float Correct(Landmark mark, /* 補正に使用するランドマークの種類 */
                float distance /* 補正前の距離 */
  );

  /* 記録したランドマークを取得 */
  const std::vector<float> &Get(Landmark mark) const;

 private:
  uint32_t curvatureIndex_;         /* 曲率マーカーのインデックス */
  std::vector<float> curvatureVec_; /* 曲率マーカー */
  uint32_t crossLineIndex_;         /* 交差のインデックス */
  std::vector<float> crossLineVec_; /* 交差 */

  /* 一番近い補正位置を取得 */
  static bool SearchNearest(const std::vector<float> &v, /* 補正位置のベクタ */
                            float distance,              /* 補正位置 */
                            float allowError,            /* 補正を許容するずれ */
                            uint32_t &minIndex,          /* 検索下限インデックス */
                            float &nearest               /* 結果 */
  );
};
}  // namespace MotionPlaning

#endif  // MOTIONPLANING_POSITIONCORRECTOR_H_
