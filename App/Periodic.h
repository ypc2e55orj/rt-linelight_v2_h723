#ifndef APP_PERIODIC_H_
#define APP_PERIODIC_H_

/* STM32CubeMX */
#include <main.h>

/* Project */
#include "Wrapper/Mutex.h"
#include "Wrapper/New.h"
#include "Wrapper/Task.h"

/* C++ */
#include <array>
#include <mutex>

class Periodic final : public Task<Periodic> {
 public:
  static constexpr uint32_t kMaxTask = 10;

  /* 初期化 */
  bool Initialize();

  /* タスクを追加 */
  bool Add(TaskHandle_t task);

  /* タスクを削除 */
  bool Remove(TaskHandle_t task);

  /* 定期通知を待機 */
  static bool WaitPeriodicNotify(TickType_t xTicksToWait = portMAX_DELAY);

 protected:
  /* タスク */
  void TaskEntry() final;

 private:
  Mutex mtx_;
  std::array<TaskHandle_t, kMaxTask> tasks_;

  /* タイマー割り込みによる通知 */
  static void PeriodElapsedCallback(TIM_HandleTypeDef *);
};

#endif  // APP_PERIODIC_H_
