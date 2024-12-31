#ifndef APP_UI_H_
#define APP_UI_H_

/* FreeRTOS */
#include <FreeRTOS.h>
#include <queue.h>

/* Project */
#include "Wrapper/Mutex.h"
#include "Wrapper/Task.h"

class Ui : public Task<Ui> {
 public:
  enum IndicatorBit {
    kIndicatorBit0, /* 0x01 */
    kIndicatorBit1, /* 0x02 */
    kIndicatorBit2, /* 0x04 */
    kIndicatorBit3, /* 0x08 */
    kIndicatorBit4, /* 0x10 */
    kIndicatorBit5, /* 0x20 */
    kIndicatorBit6, /* 0x40 */
  };
  enum IndicatorMask {
    kIndicatorMask0 = (1 << kIndicatorBit0),
    kIndicatorMask1 = (1 << kIndicatorBit1),
    kIndicatorMask2 = (1 << kIndicatorBit2),
    kIndicatorMask3 = (1 << kIndicatorBit3),
    kIndicatorMask4 = (1 << kIndicatorBit4),
    kIndicatorMask5 = (1 << kIndicatorBit5),
    kIndicatorMask6 = (1 << kIndicatorBit6),
  };

  /* コンストラクタ */
  Ui();

  /* UIタスクを作成 */
  bool Initialize();

  /* インジケータを設定 */
  bool SetIndicator(uint8_t pos, uint8_t mask);

  /* ブザーを鳴動 */
  bool SetBuzzer(uint16_t freq, uint16_t duration);

  /* ボタン押下時間を取得 */
  uint32_t WaitPress(TickType_t xTicksToWait = portMAX_DELAY);

  /* 警告 */
  void Warn(int times = 3);

  /* 緊急表示 */
  void Fatal();

 protected:
  /* タスク */
  void TaskEntry() final;

 private:
  static constexpr uint32_t kUiUpdateInterval = 20; /* 状態更新周期 [ms] */

  /* 緊急表示 */
  Mutex uiMtx_;

  /* LED */
  bool SetIndicator_(uint8_t pos, uint8_t mask);
  struct Indicator {
    uint8_t pos, mask;
  };
  uint8_t ledQueueStorageBuffer_[sizeof(Indicator)];
  StaticQueue_t ledQueueBuffer_;
  QueueHandle_t ledQueue_;
  void UpdateLed();

  /* ブザー */
  bool SetBuzzer_(uint16_t freq, uint16_t duration);
  struct Buzzer {
    uint32_t toneFreq;
    uint32_t duration;
  };
  struct {
    uint32_t pwmFreq;
    uint32_t remainDuration;
  } buzzer_;
  uint8_t buzzerQueueStorageBuffer_[sizeof(Buzzer)];
  StaticQueue_t buzzerQueueBuffer_;
  QueueHandle_t buzzerQueue_;
  void UpdateBuzzer();

  /* ボタン */
  static constexpr uint32_t kButtonPressThreshold = 5; /* ボタン押下検出閾値 [20ms] */
  using Button = uint32_t;
  struct {
    bool prevPressed;
    uint32_t pressTotal;
  } button_;
  uint8_t buttonQueueStorageBuffer_[sizeof(Button)];
  StaticQueue_t buttonQueueBuffer_;
  QueueHandle_t buttonQueue_;
  void UpdateButton();
};
#endif  // APP_UI_H_
