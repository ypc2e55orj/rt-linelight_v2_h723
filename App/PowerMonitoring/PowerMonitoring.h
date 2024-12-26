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
  const Power &GetPower() { return power_; }

 protected:
  /* タスク */
  void TaskEntry() final;

 private:
  Power power_;
};
}  // namespace PowerMonitoring
#endif  // POWERMONITORING_POWERMONITORING_H_
