#ifndef POWERMONITORING_POWERMONITORING_H_
#define POWERMONITORING_POWERMONITORING_H_

/* Projects */
#include "Config.h"
#include "Data/MovingAverage.h"
#include "PowerMonitoring/Power.h"
#include "Wrapper/Task.h"

namespace PowerMonitoring {
class PowerMonitoring final : public Task<PowerMonitoring> {
 public:
  /* 初期化 */
  bool Initialize();

  /* 状態を取得 */
  const PowerImpl &Power() { return power_; }

 protected:
  /* タスク */
  void TaskEntry() final;

 private:
  PowerImpl power_;
};
}  // namespace PowerMonitoring
#endif  // POWERMONITORING_POWERMONITORING_H_
