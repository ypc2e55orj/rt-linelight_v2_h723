#ifndef MOTIONPLANING_MOTIONPLANING_H_
#define MOTIONPLANING_MOTIONPLANING_H_

/* Projects */
#include "MotionPlaning/Motor.h"
#include "MotionPlaning/Servo.h"
#include "Wrapper/Task.h"

namespace MotionPlaning {
class MotionPlaning final : public Task<MotionPlaning> {
 public:
  /* コンストラクタ */
  MotionPlaning();

  /* 初期化 */
  bool Initialize();

  /* サーボを取得 */
  Servo &GetServo() { return servo_; }

 protected:
  /* タスク */
  void TaskEntry() final;

 private:
  Servo servo_;
};
}  // namespace MotionPlaning

#endif  // MOTIONPLANING_MOTIONPLANING_H_
