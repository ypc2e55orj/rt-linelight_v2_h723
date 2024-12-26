#include "App.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"

#define UNIT_TEST_LINESENSING

#ifdef APP_UNIT_TEST
#if defined(UNIT_TEST_UI)
/**
 * MARK: Unit (Ui)
 */
#include "Ui.h"

extern "C" void vAPP_TaskEntry() {
  Ui::Instance().Initialize();
  uint32_t count = 0;
  while (true) {
    if (++count >= 10) {
      count = 0;
      Ui::Instance().SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
    }
    Ui::Instance().SetIndicator(count > 5 ? 0xff : 0x00, 0xff);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
#elif defined(UNIT_TEST_PERIODIC)
/**
 * MARK: Unit (Periodic)
 */
#include "Periodic.h"
#include "Ui.h"

extern "C" void vAPP_TaskEntry() {
  uint32_t notify = 0;
  uint32_t count = 0;
  Ui::Instance().Initialize();
  if (!Periodic::Instance().Initialize()) {
    Ui::Instance().Fatal();
  }
  Periodic::Instance().Add(xTaskGetCurrentTaskHandle());
  while (true) {
    if (xTaskNotifyWait(0, kTaskNotifyBitMask, &notify, portMAX_DELAY) == pdTRUE) {
      if (notify & kTaskNotifyBitPeriodic) {
        if (++count >= 1000) {
          count = 0;
          Ui::Instance().SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
        }
        Ui::Instance().SetIndicator(count > 500 ? 0xff : 0x00, 0xff);
      }
    }
  }
}
#elif defined(UNIT_TEST_COM)
/**
 * MARK: Unit (Com)
 */
#include "Com.h"
#include "Periodic.h"
#include "Ui.h"

extern "C" void vAPP_TaskEntry() {
  Ui::Instance().Initialize();

  if (!Com::Instance().Initialize()) {
    Ui::Instance().Fatal();
  }
  if (!Periodic::Instance().Initialize()) {
    Ui::Instance().Fatal();
  }
  uint32_t notify = 0;
  uint32_t count = 0;
  Periodic::Instance().Add(xTaskGetCurrentTaskHandle());
  while (true) {
    if (xTaskNotifyWait(0, kTaskNotifyBitMask, &notify, portMAX_DELAY) == pdTRUE) {
      if (notify & kTaskNotifyBitPeriodic) {
        if (++count >= 1000) {
          count = 0;
          printf("rt-linelight Unit test (Com)! %ld\r\n", HAL_GetTick());
        }
      }
    }
  }
}
#elif defined(UNIT_TEST_FRAM)
/**
 * MARK: Unit (Fram)
 */
#include "Com.h"
#include "Fram.h"
#include "Ui.h"

extern "C" void vAPP_TaskEntry() {
  Ui::Instance().Initialize();
  if (!Com::Instance().Initialize()) {
    Ui::Instance().Fatal();
  }
  if (!Fram::Instance().Initialize()) {
    Ui::Instance().Fatal();
  }

  uint32_t chunk[64] = {};
  auto &fram = Fram::Instance();
  auto &ui = Ui::Instance();

  for (uint32_t address = 0; (address + sizeof(chunk)) < Fram::kMaxAddress; address += sizeof(chunk)) {
    for (uint32_t i = 0; i < sizeof(chunk) / sizeof(chunk[0]); i++) {
      chunk[i] = address + i;
    }
    if (!fram.Write(address, chunk, sizeof(chunk))) {
      printf("Write error at %lx\r\n", address);
      ui.Fatal();
    }
  }
  for (uint32_t address = 0; (address + sizeof(chunk)) < Fram::kMaxAddress; address += sizeof(chunk)) {
    memset(chunk, 0, sizeof(chunk));
    if (!fram.Read(address, chunk, sizeof(chunk))) {
      printf("Read error at %lx\r\n", address);
      ui.Fatal();
    }
    for (uint32_t i = 0; i < sizeof(chunk) / sizeof(chunk[0]); i++) {
      if (chunk[i] != (address + i)) {
        printf("Validate error at %lx\r\n", address + i);
        ui.Fatal();
      }
      printf("%lx == %lx\r\n", address + i, chunk[i]);
    }
  }
  vTaskDelay(portMAX_DELAY);
}
#elif defined(UNIT_TEST_POWERMONITORING)
/**
 * MARK: Unit (PowerMonitoring)
 */
#include "Com.h"
#include "PowerMonitoring/PowerMonitoring.h"
#include "Ui.h"

extern "C" void vAPP_TaskEntry() {
  Ui::Instance().Initialize();
  if (!Com::Instance().Initialize()) {
    Ui::Instance().Fatal();
  }
  if (!PowerMonitoring::PowerMonitoring::Instance().Initialize()) {
    Ui::Instance().Fatal();
  }
  printf("\r\n\r\nReset!!\r\n\r\n");
  auto &power = PowerMonitoring::PowerMonitoring::Instance().GetPower();
  while (true) {
    auto vbatt = power.GetBatteryVoltage();
    auto vbattAvg = power.GetBatteryVoltageAverage();
    auto errorCount = power.GetBatteryErrorCount();
    printf("%ld, %f, %f, %ld\r\n", HAL_GetTick(), static_cast<double>(vbatt), static_cast<double>(vbattAvg),
           errorCount);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
#elif defined(UNIT_TEST_MOTIONSENSING)
/**
 * MARK: Unit (MotionSensing)
 */
#include "Com.h"
#include "MotionSensing/Imu.h"
#include "MotionSensing/MotionSensing.h"
#include "Periodic.h"
#include "Ui.h"

extern "C" void vAPP_TaskEntry() {
  uint32_t notify = 0;
  uint32_t count = 0;

  Ui::Instance().Initialize();
  if (!Com::Instance().Initialize()) {
    Ui::Instance().Fatal();
  }
  if (!Periodic::Instance().Initialize()) {
    Ui::Instance().Fatal();
  }
  if (!MotionSensing::MotionSensing::Instance().Initialize()) {
    Ui::Instance().Fatal();
  }
  Periodic::Instance().Add(xTaskGetCurrentTaskHandle());

  auto &sensing = MotionSensing::MotionSensing::Instance();
  auto &imu = MotionSensing::Imu::Instance();

  vTaskDelay(pdMS_TO_TICKS(1000)); /* 起動してしばらくはIMUが安定していないので */

  sensing.CalibrateImu(500);

  sensing.NotifyStart();
  while (true) {
    if (xTaskNotifyWait(0, kTaskNotifyBitMask, &notify, portMAX_DELAY) == pdTRUE) {
      if (notify & kTaskNotifyBitPeriodic) {
        if (++count >= 1000) {
          count = 0;
          auto acc = imu.GetAccelRaw();
          auto gyro = imu.GetGyroRaw();
          printf("%ld, %d, %d, %d, %d, %d, %d\r\n", HAL_GetTick(), gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2]);
        }
      }
    }
  }
}
#elif defined(UNIT_TEST_LINESENSING)
/**
 * MARK: Unit (LineSensing)
 */
#include "Com.h"
#include "LineSensing/LineSensing.h"
#include "Periodic.h"
#include "Ui.h"

extern "C" void vAPP_TaskEntry() {
  uint32_t notify = 0;
  uint32_t count = 0;

  Ui::Instance().Initialize();
  auto &ui = Ui::Instance();

  if (!Com::Instance().Initialize()) {
    ui.Fatal();
  }
  if (!Periodic::Instance().Initialize()) {
    ui.Fatal();
  }
  if (!LineSensing::LineSensing::Instance().Initialize()) {
    ui.Fatal();
  }
  auto &sensing = LineSensing::LineSensing::Instance();

  /* キャリブレーション */
  ui.WaitPress();
  ui.SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
  sensing.Calibrate(2500);

  auto &line = sensing.GetLine();
  LineSensing::Line::Raw raw{};
  sensing.NotifyStart();

  Periodic::Instance().Add(xTaskGetCurrentTaskHandle());
  while (true) {
    if (xTaskNotifyWait(0, kTaskNotifyBitMask, &notify, portMAX_DELAY) == pdTRUE) {
      if (notify & kTaskNotifyBitPeriodic) {
        line.GetRaw(raw);
        for (uint32_t i = 0; i < LineSensing::Line::kNum; i++) {
          printf("%d, ", raw[i]);
        }
        printf("\r\n");
      }
    }
  }
}

#endif  // UNIT_TEST_*
#endif  // APP_UNIT_TEST

#pragma GCC diagnostic pop
