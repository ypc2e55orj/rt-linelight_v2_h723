#include "Periodic.h"

/* Project */
#include "Config.h"

/* C++ */
#include <algorithm>

/* グローバル変数定義 */
extern TIM_HandleTypeDef htim23; /* 1kHz (STM32CubeMXで設定) */

/* 初期化 */
bool Periodic::Initialize() {
  for (auto &t : tasks_) {
    t = nullptr;
  }
  if (HAL_TIM_RegisterCallback(&htim23, HAL_TIM_PERIOD_ELAPSED_CB_ID, PeriodElapsedCallback) != HAL_OK) {
    return false;
  }
  return TaskCreate("Periodic", configMINIMAL_STACK_SIZE, kPriorityPeriodic);
}

/* タスクを追加 */
bool Periodic::Add(TaskHandle_t task) {
  std::scoped_lock<Mutex> lock(mtx_);
  for (auto &t : tasks_) {
    if (t == nullptr) {
      t = task;
      return true;
    }
  }
  return false;
}

/* タスクを削除 */
bool Periodic::Remove(TaskHandle_t task) {
  std::scoped_lock<Mutex> lock(mtx_);
  for (auto &t : tasks_) {
    if (t == task) {
      t = nullptr;
      return true;
    }
  }
  return false;
}

/* タスク */
void Periodic::TaskEntry() {
  HAL_TIM_Base_Start_IT(&htim23);
  uint32_t notify = 0;
  while (true) {
    /* タイマー割り込み待ち */
    /* TODO: タイムアウト */
    if (!TaskNotifyWait(notify)) {
      /* TODO: エラーハンドリング */
    }
    if ((notify & kTaskNotifyBitPeriodic) == kTaskNotifyBitPeriodic) {
      std::scoped_lock<Mutex> lock(mtx_);
      for (auto task : tasks_) {
        xTaskNotify(task, kTaskNotifyBitPeriodic, eSetBits);
      }
    }
  }
}

/* タイマー割り込みによる通知 */
void Periodic::PeriodElapsedCallback(TIM_HandleTypeDef *) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xTaskNotifyFromISR(Instance().TaskHandle(), kTaskNotifyBitPeriodic, eSetBits, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* 定期通知を待機 */
bool Periodic::WaitPeriodicNotify(TickType_t xTicksToWait) {
  uint32_t notify = 0;
  if (xTaskNotifyWait(0, kTaskNotifyBitMask, &notify, xTicksToWait) != pdTRUE) {
    return false;
  }
  if ((notify & kTaskNotifyBitPeriodic) == kTaskNotifyBitPeriodic) {
    return true;
  }
  return false;
}
