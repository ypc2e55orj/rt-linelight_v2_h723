#ifndef MOTIONPLANING_MOTOR_H_
#define MOTIONPLANING_MOTOR_H_

/* Project */
#include "Data/Singleton.h"

/* C++ */
#include <array>

namespace MotionPlaning {
class Motor final : public Singleton<Motor> {
 public:
  using Duty = std::array<float, 2>;

  /* 初期化 */
  bool Initialize();

  /* 有効化 */
  void Enable();
  void Disable();

  /* フォールトを取得 */
  bool IsFault();

  /* フリーに設定 */
  void SetFree();

  /* デューティを設定 */
  void SetDuty(const Duty &duty);
};
}  // namespace MotionPlaning

#endif  // MOTIONPLANING_MOTOR_H_
