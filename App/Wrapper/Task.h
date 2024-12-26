#ifndef WRAPPER_TASK_H_
#define WRAPPER_TASK_H_

/* FreeRTOS */
#include <FreeRTOS.h>
#include <semphr.h>

/* Project */
#include "Data/Singleton.h"

/* 共通のタスク通知値 */
enum TaskNotifyBit {
  kTaskNotifyBitStart = (0x01 << 0),
  kTaskNotifyBitStop = (0x01 << 1),
  kTaskNotifyBitPeriodic = (0x01 << 2),
  kTaskNotifyBitLast = kTaskNotifyBitPeriodic,
};
static constexpr uint32_t kTaskNotifyBitMask = (kTaskNotifyBitLast << 1) - 1;

/**
 * タスク基底クラス
 */
template <typename T>
class Task : public Singleton<T> {
 public:
  /* 開始通知 */
  bool NotifyStart() { return xTaskNotify(task_, kTaskNotifyBitStart, eSetBits) == pdPASS; }
  /* 停止通知 */
  bool NotifyStop() { return xTaskNotify(task_, kTaskNotifyBitStop, eSetBits) == pdPASS; }

 protected:
  /* タスク作成 */
  virtual bool TaskCreate(const char *const pcName, uint16_t usStackDepth, UBaseType_t uxPriority) {
    return xTaskCreate([](void *p) { reinterpret_cast<decltype(this)>(p)->TaskEntry(); }, pcName, usStackDepth, this,
                       uxPriority, &task_) == pdPASS;
  }

  /* タスク */
  virtual void TaskEntry() = 0;

  /* タスクハンドルを取得 */
  TaskHandle_t TaskHandle() const { return task_; }

  /* 通知を待つ */
  static bool TaskNotifyWait(uint32_t &notify, TickType_t xTicksToWait = portMAX_DELAY) {
    notify = 0;
    return xTaskNotifyWait(0, kTaskNotifyBitMask, &notify, xTicksToWait) == pdPASS;
  }
  /* 開始通知を待つ */
  static bool TaskNotifyWaitStart(TickType_t xTicksToWait = portMAX_DELAY) {
    uint32_t notify = 0;
    while (true) {
      if (!TaskNotifyWait(notify, xTicksToWait)) {
        return false; /* タイムアウト */
      }
      if (notify & kTaskNotifyBitStart) {
        return true;
      }
    }
  }

 private:
  TaskHandle_t task_{nullptr};
};

#endif  // WRAPPER_TASK_H_
