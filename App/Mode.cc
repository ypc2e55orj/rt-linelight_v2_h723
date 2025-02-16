#include "Mode.h"

/* Project */
#include "Config.h"
#include "MotionSensing/MotionSensing.h"
#include "Periodic.h"
#include "Ui.h"

static uint8_t SelectModeLoop(uint8_t mask, uint8_t prev) {
  auto &ui = Ui::Instance();
  auto &odometry = MotionSensing::MotionSensing::Instance().Odometry();
  uint32_t pressTime = 0;
  float velo = 0.0f;
  uint8_t mode = prev;
  ui.SetIndicator(mode, mask);
  /* モード選択ループ */
  auto xLastWakeTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    pressTime = ui.WaitPress(0);
    velo = odometry.GetVelocity().trans;

    /* モード仮確定 */
    if (mode != 0 && pressTime >= kButtonLongPressThreshold) {
      ui.SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
      bool blink = false;
      while (true) {
        pressTime = ui.WaitPress(0);
        if (pressTime >= kButtonLongPressThreshold) { /* キャンセル */
          ui.SetBuzzer(kBuzzerFrequency, kBuzzerCancelDuration);
          break;
        } else if (pressTime >= kButtonShortPressThreshold) { /* 確定 */
          ui.SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
          ui.SetIndicator(mode, mask);
          vTaskDelay(pdMS_TO_TICKS(200));
          return mode;
        }
        /* インジケータ点滅 */
        ui.SetIndicator(blink ? mode : 0x00, mask);
        blink = !blink;
        vTaskDelay(pdMS_TO_TICKS(50));
      }
    }
    /* 車輪が一定速度以上で回ったかボタンが短押された場合モード変更 */
    else if (std::abs(velo) > kModeSelectWheelSpeed || pressTime >= kButtonShortPressThreshold) {
      if (std::signbit(velo)) {
        mode = (mode - 1) & mask;
      } else {
        mode = (mode + 1) & mask;
      }
      ui.SetIndicator(mode, mask);
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

/* モード選択 */
uint8_t SelectMode(uint8_t mask, uint8_t prev) {
  MotionSensing::MotionSensing::Instance().NotifyStart();
  auto mode = SelectModeLoop(mask, prev);
  MotionSensing::MotionSensing::Instance().NotifyStop();
  return mode;
}
