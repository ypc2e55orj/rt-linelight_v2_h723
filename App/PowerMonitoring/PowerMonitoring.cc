#include "PowerMonitoring/PowerMonitoring.h"

/* STM32CubeMX */
#include <main.h>

/* C++ */
#include <mutex>

/* Project */
#include "Config.h"
#include "Periodic.h"
#include "PowerMonitoring/PowerAdc.h"

namespace PowerMonitoring {

/* 初期化 */
bool PowerMonitoring::Initialize() {
  /* ADCを初期化 */
  if (!PowerAdc::Instance().Initialize()) {
    return false;
  }
  return TaskCreate("PowerMonitoring", configMINIMAL_STACK_SIZE, kPriorityPowerMonitoring);
}

/* タスク */
void PowerMonitoring::TaskEntry() {
  uint32_t notify = 0;
  power_.Reset();
  Periodic::Instance().Add(TaskHandle());
  while (true) {
    /* TODO: タイムアウト */
    if (!TaskNotifyWait(notify)) {
      /* TODO: エラーハンドリング */
    }
    if (notify & kTaskNotifyBitPeriodic) {
      /* 電源監視 */
      if (!power_.Update()) {
        /* TODO: エラー時 */
      }
      if (power_.GetAdcErrorTime() >= kPowerAdcErrorTime || power_.GetBatteryErrorTime() >= kBatteryErrorTime) {
        /* 一定時間以上異常の場合はリセット */
        NVIC_SystemReset();
      }
    }
  }
}
}  // namespace PowerMonitoring
