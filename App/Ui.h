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
    kIndicator1Bit,     /* 0x01 */
    kIndicator2Bit,     /* 0x02 */
    kIndicator3Bit,     /* 0x04 */
    kIndicator4Bit,     /* 0x08 */
    kIndicator5Bit,     /* 0x10 */
    kIndicatorLeftBit,  /* 0x20 */
    kIndicatorRightBit, /* 0x40 */
  };
  enum IndicatorMask {
    kIndicator1Mask = (1 << kIndicator1Bit),
    kIndicator2Mask = (1 << kIndicator2Bit),
    kIndicator3Mask = (1 << kIndicator3Bit),
    kIndicator4Mask = (1 << kIndicator4Bit),
    kIndicator5Mask = (1 << kIndicator5Bit),
    kIndicatorLeftMask = (1 << kIndicatorLeftBit),
    kIndicatorRightMask = (1 << kIndicatorRightBit),
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
