#include "Ui.h"

/* STM32CubeMX */
#include <main.h>

/* Project */
#include "config.h"

/* C++ */
#include <mutex>

// #define UI_FORCE_MUTE /* 強制ミュート */

/* 外部変数 */
extern TIM_HandleTypeDef htim1;

/* コンストラクタ */
Ui::Ui() {
  ledQueue_ = xQueueCreateStatic(1, sizeof(Indicator), ledQueueStorageBuffer_, &ledQueueBuffer_);
  buzzerQueue_ = xQueueCreateStatic(1, sizeof(Buzzer), buzzerQueueStorageBuffer_, &buzzerQueueBuffer_);
  buttonQueue_ = xQueueCreateStatic(1, sizeof(Button), buttonQueueStorageBuffer_, &buttonQueueBuffer_);
  buzzer_.pwmFreq = (HAL_RCC_GetPCLK1Freq() * 2) / (htim1.Init.Prescaler + 1);
}

/* UIタスクを作成 */
bool Ui::Initialize() { return TaskCreate("Ui", 256, kPriorityUi); }

/* インジケータを設定 */
bool Ui::SetIndicator_(uint8_t pos, uint8_t mask) {
  Indicator msg = {pos, mask};
  return xQueueOverwrite(ledQueue_, &msg) == pdTRUE;
}
bool Ui::SetIndicator(uint8_t pos, uint8_t mask) {
  std::scoped_lock<Mutex> lock(uiMtx_);
  return SetIndicator_(pos, mask);
}

/* ブザーを鳴動 */
bool Ui::SetBuzzer_(uint16_t freq, uint16_t duration) {
  Buzzer msg = {freq, duration};
  return xQueueSend(buzzerQueue_, &msg, portMAX_DELAY) == pdTRUE;
}
bool Ui::SetBuzzer(uint16_t freq, uint16_t duration) {
  std::scoped_lock<Mutex> lock(uiMtx_);
  return SetBuzzer_(freq, duration);
}

/* ボタン押下時間を取得 */
uint32_t Ui::WaitPress(TickType_t xTicksToWait) {
  uint32_t msg = 0;
  if (xQueueReceive(buttonQueue_, &msg, xTicksToWait) == pdTRUE) {
    return msg;
  }
  return 0;
}

/* 警告 */
void Ui::Warn(int times) {
  std::scoped_lock<Mutex> lock(uiMtx_);
  bool blink = false;
  for (int t = 0; t < times; t++) {
    SetIndicator_(blink ? 0x00 : 0x1f, 0x1f);
    SetBuzzer_(4000, 100);
    blink = !blink;
    vTaskDelay(pdMS_TO_TICKS(200));
  }
  SetIndicator_(0x00, 0xff);
}

/* 緊急表示 */
void Ui::Fatal() {
  bool blink = false;
  uiMtx_.lock(); /* 解放しない */
  while (true) {
    SetIndicator_(blink ? 0x00 : 0xff, 0xff);
    SetBuzzer_(4000, 100);
    blink = !blink;
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

/* タスク */
void Ui::TaskEntry() {
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  auto xLastWakeTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(kUiUpdateInterval));
    UpdateLed();
    UpdateBuzzer();
    UpdateButton();
  }
  /* 停止させることはないはず */
}

/* LED */
void Ui::UpdateLed() {
  Indicator msg = {};
  if (xQueueReceive(ledQueue_, &msg, 0) == pdTRUE) {
    if (msg.mask & kIndicatorRightMask)
      HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin,
                        (msg.pos & kIndicatorRightMask) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if (msg.mask & kIndicatorLeftMask)
      HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin,
                        (msg.pos & kIndicatorLeftMask) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if (msg.mask & kIndicator1Mask)
      HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, (msg.pos & kIndicator1Mask) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if (msg.mask & kIndicator2Mask)
      HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, (msg.pos & kIndicator2Mask) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if (msg.mask & kIndicator3Mask)
      HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, (msg.pos & kIndicator3Mask) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if (msg.mask & kIndicator4Mask)
      HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, (msg.pos & kIndicator4Mask) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if (msg.mask & kIndicator5Mask)
      HAL_GPIO_WritePin(LED_5_GPIO_Port, LED_5_Pin, (msg.pos & kIndicator5Mask) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
}

/* ブザー */
void Ui::UpdateBuzzer() {
  /* キューから設定を取得 */
  Buzzer msg = {};

  /* 鳴動時間が経過したら停止 */
  if (buzzer_.remainDuration > 0) {
    buzzer_.remainDuration--;
  } else {
    if (xQueueReceive(buzzerQueue_, &msg, 0) == pdTRUE) {
      /* 周波数を設定 */
      buzzer_.remainDuration = msg.duration / kUiUpdateInterval;
#ifndef UI_FORCE_MUTE
      uint16_t period = static_cast<uint16_t>(buzzer_.pwmFreq / msg.toneFreq);
      __HAL_TIM_SET_AUTORELOAD(&htim1, period);
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, period / 2);
#endif  // UI_FORCE_MUTE
    } else {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    }
  }
}

/* ボタン */
void Ui::UpdateButton() {
  bool pressed = HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_RESET;

  /* 離されたらキューに追加 */
  if (button_.prevPressed && !pressed) {
    if (button_.pressTotal >= kButtonPressThreshold) {
      uint32_t ms = button_.pressTotal * kUiUpdateInterval;
      xQueueSend(buttonQueue_, &ms, 0);
    }
  }
  /* 押下時間をインクリメント */
  if (pressed) {
    button_.pressTotal++;
  } else {
    button_.pressTotal = 0;
  }

  button_.prevPressed = pressed;
}
