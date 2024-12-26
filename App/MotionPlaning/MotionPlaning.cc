#include "MotionPlaning/MotionPlaning.h"

/* Projects */
#include "MotionPlaning/Motor.h"
#include "MotionPlaning/Servo.h"
#include "MotionSensing/MotionSensing.h"
#include "Periodic.h"
#include "PowerMonitoring/PowerMonitoring.h"

namespace MotionPlaning {
/* コンストラクタ */
MotionPlaning::MotionPlaning() : servo_() {}

/* 初期化 */
bool MotionPlaning::Initialize() {
  /* モーター初期化 */
  if (!Motor::Instance().Initialize()) {
    return false;
  }
  return TaskCreate("MotionPlaning", configMINIMAL_STACK_SIZE, kPriorityMotionPlaning);
}

/* タスク */
void MotionPlaning::TaskEntry() {
  uint32_t notify = 0;
  auto &motor = Motor::Instance();
  auto &power = PowerMonitoring::PowerMonitoring::Instance().GetPower();
  auto &odometry = MotionSensing::MotionSensing::Instance().GetOdometry();
  Periodic::Instance().Add(TaskHandle());
  while (true) {
    TaskNotifyWaitStart();
    servo_.Reset();
    motor.Enable();
    while (true) {
      /* TODO: タイムアウト */
      if (!TaskNotifyWait(notify)) {
        /* TODO: エラーハンドリング */
      }
      if (notify & kTaskNotifyBitStop) {
        motor.SetFree();
        motor.Disable();
        break;
      }
      if (notify & kTaskNotifyBitPeriodic) {
        float batteryVoltage = power.GetBatteryVoltage();
        auto acc = odometry.GetAcceleration();
        auto velo = odometry.GetVelocity();
        servo_.Update(batteryVoltage, velo.trans, acc.trans, velo.rot, acc.rot);
        motor.SetDuty(servo_.GetMotorDuty());
      }
    }
  }
}
}  // namespace MotionPlaning
