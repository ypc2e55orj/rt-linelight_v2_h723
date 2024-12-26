#ifndef MOTIONPLANING_SUCTION_H_
#define MOTIONPLANING_SUCTION_H_

/* Project */
#include "Data/Singleton.h"

/* C++ */
#include <array>

namespace MotionPlaning {
class Suction final : public Singleton<Suction> {
 public:
  /* 有効化 */
  bool Enable();
  /* 無効化 */
  bool Disable();
  /* デューティを設定 */
  void SetDuty(float duty);
};
}  // namespace MotionPlaning

#endif  // MOTIONPLANING_SUCTION_H_
