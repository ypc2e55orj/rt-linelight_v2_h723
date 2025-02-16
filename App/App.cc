#include "App.h"

/* C++ */
#include <cstdio>
#include <cstring>

/* Project */
#include "Com.h"
#include "Fram.h"
#include "LineSensing/LineSensing.h"
#include "Mode.h"
#include "MotionPlaning/MotionPlaning.h"
#include "MotionSensing/MotionSensing.h"
#include "Periodic.h"
#include "PowerMonitoring/PowerMonitoring.h"
#include "Test.h"
#include "Trace.h"
#include "Ui.h"

#ifndef APP_UNIT_TEST
/**
 * MARK: Initialize
 * 初期化
 */
static void Initialize() {
  vTaskDelay(pdMS_TO_TICKS(1000)); /* 電圧が上がり切るまで待つ */
  Ui::Instance().Initialize();
  if (!Com::Instance().Initialize()) {
    Ui::Instance().Fatal();
  }
  printf("FRAM... ");
  if (Fram::Instance().Initialize()) {
    printf("OK\r\n");
  } else {
    Ui::Instance().Fatal();
    printf("NG\r\n");
  }

  /* ここの順序はPeriodicクラスの呼び出し順になるので注意 */
  printf("Periodic... ");
  if (Periodic::Instance().Initialize()) {
    printf("OK\r\n");
  } else {
    Ui::Instance().Fatal();
    printf("NG\r\n");
  }
  printf("Power Monitoring... ");
  if (PowerMonitoring::PowerMonitoring::Instance().Initialize()) {
    printf("OK\r\n");
  } else {
    printf("NG\r\n");
    Ui::Instance().Fatal();
  }
  printf("Motion Sensing... ");
  if (MotionSensing::MotionSensing::Instance().Initialize()) {
    printf("OK\r\n");
  } else {
    printf("NG\r\n");
    Ui::Instance().Fatal();
  }
  printf("Line Sensing... ");
  if (LineSensing::LineSensing::Instance().Initialize()) {
    printf("OK\r\n");
  } else {
    printf("NG\r\n");
    Ui::Instance().Fatal();
  }
  printf("Motion Planing... ");
  if (MotionPlaning::MotionPlaning::Instance().Initialize()) {
    printf("OK\r\n");
  } else {
    printf("NG\r\n");
    Ui::Instance().Fatal();
  }
  /* 一番最後に */
  Periodic::Instance().Add(xTaskGetCurrentTaskHandle());
}

/**
 * MARK: ShowBatteryVoltage
 * 起動時の電圧確認
 */
static void ShowBatteryVoltage() {
  auto &power = PowerMonitoring::PowerMonitoring::Instance().Power();
  auto &ui = Ui::Instance();
  auto volt = power.GetBatteryVoltageAverage();
  if (volt < kBatteryVoltageLimitMin) {
    printf("Battery voltage is too low! (%f V)\r\n", static_cast<double>(volt));
    ui.Fatal(); /* ここで停止 */
  }

  float batteryLevelDiv = (kBatteryVoltageLimitMax - kBatteryVoltageLimitMin) / 5.0f;
  float batteryLevel = volt - kBatteryVoltageLimitMin;
  uint8_t indicatorLevel = 0;
  for (int i = 0; i < 5; i++) {
    if (batteryLevel > (static_cast<float>(i + 1) * batteryLevelDiv)) {
      indicatorLevel |= 1 << i;
    }
  }
  ui.SetIndicator(indicatorLevel, 0x1f);
  printf("rt-linelight(%s %s) is started! (%f V, %ld)\r\n", __DATE__, __TIME__, static_cast<double>(volt),
         power.GetTick());
}

/**
 * MARK: Calibrate
 * ラインセンサーのキャリブレーション
 */
static void Calibrate() {
  auto &ui = Ui::Instance();
  ui.WaitPress();
  ui.SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
  if (!LineSensing::LineSensing::Instance().Calibrate(2000)) {
    ui.Fatal();
  }
  ui.SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
}

extern "C" void vAPP_TaskEntry() {
  Initialize();
  ShowBatteryVoltage();

  /* スイッチから手が離れるまで待つ */
  auto &ui = Ui::Instance();
  ui.SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
  vTaskDelay(1000);
  ui.SetIndicator(0x00, 0xff);

  Calibrate();

  uint8_t mode = 0;
  auto &trace = Trace::Instance();
  /* モード選択 */
  while (true) {
    mode = SelectMode(0x1f, mode);
    switch (mode) {
      case 0x01: {
        Trace::Parameter param = {
            Trace::Mode::kExploreRunning, /* モード */
            10,                           /* ログ周期 [ms] */
            1.0f,                         /* 探索速度[m/s] */
            1.0f,                         /* 加速度 [m/ss] */
            {5.0f, 0.01f, 0.0f},          /* 並進PIDゲイン */
            {0.3f, 0.05f, 0.0f},          /* 旋回PIDゲイン */
            {26.0f, 0.0f, 0.02f},         /* ライン追従PIDゲイン */
            0.1f,                         /* ゴールマーカーから停止までの距離 [m] */
            1.0f,                         /* 吸引電圧 [V] */
        };
        trace.Run(param);
      } break;
      case 0x02: {
        Trace::Parameter param = {
            Trace::Mode::kExploreRunning, /* モード */
            10,                           /* ログ周期 [ms] */
            1.5f,                         /* 探索速度[m/s] */
            5.0f,                         /* 加速度 [m/ss] */
            {5.0f, 0.01f, 0.0f},          /* 並進PIDゲイン */
            {0.3f, 0.05f, 0.0f},          /* 旋回PIDゲイン */
            {26.0f, 0.0f, 0.02f},         /* ライン追従PIDゲイン */
            0.1f,                         /* ゴールマーカーから停止までの距離 [m] */
            2.5f,                         /* 吸引電圧 [V] */
        };
        trace.Run(param);
      } break;
      case 0x03: {
        Trace::Parameter param = {
            Trace::Mode::kFastRunning, /* モード */
            5,                         /* ログ周期 [ms] */
            1.0f,                      /* 開始速度[m/s] */
            5.0f,                      /* 加速度 [m/ss] */
            {5.0f, 0.01f, 0.0f},       /* 並進PIDゲイン */
            {0.3f, 0.05f, 0.0f},       /* 旋回PIDゲイン */
            {26.0f, 0.0f, 0.02f},      /* ライン追従PIDゲイン */
            0.1f,                      /* ゴールマーカーから停止までの距離 [m] */
            3.0f,                      /* 吸引電圧 [V] */
        };
        std::vector<Trace::RadiusVelocityLimit> limits{
            {0.2f, 1.2f}, {0.4f, 1.4f}, {0.6f, 1.6f}, {0.8f, 1.8f}, {1.0f, 2.0f},
        };
        // Trace::Parameter param = {
        //     Trace::Mode::kFastRunning, /* モード */
        //     10,                        /* ログ周期 [ms] */
        //     0.6f,                      /* 開始速度[m/s] */
        //     1.0f,                      /* 加速度 [m/ss] */
        //     {5.0f, 0.01f, 0.0f},       /* 並進PIDゲイン */
        //     {0.3f, 0.05f, 0.0f},       /* 旋回PIDゲイン */
        //     {26.0f, 0.0f, 0.02f},      /* ライン追従PIDゲイン */
        //     0.1f,                      /* ゴールマーカーから停止までの減速距離 [m] */
        //     0.0f,                      /* 吸引電圧 [V] */
        // };
        // std::vector<Trace::RadiusVelocityLimit> limits{
        //     {0.2f, 0.6f}, /* <R20 , 0.6m/s */
        //     {0.4f, 0.7f}, /* <R40 , 0.7m/s */
        //     {0.6f, 0.8f}, /* <R60 , 0.8m/s */
        //     {0.8f, 0.9f}, /* <R80 , 0.9m/s */
        //     {1.0f, 1.0f}, /* <R100, 1.0m/s */
        trace.CalculateVelocityMap(limits, param.limitVelocity, param.acceleration, param.acceleration, 10);
        trace.Run(param);
      } break;
      case 0x08:
        trace.PrintLog();
        break;
      case 0x09:
        trace.PrintRadiusExplorerLog();
        break;
      case 0x0a: {
        trace.PrintRadiusVelocityLog();
      } break;
      case 0x0b: {
        trace.PrintPositionCorrectorLog();
      } break;
      case 0x0c: {
        auto &line = LineSensing::LineSensing::Instance().Line();
        auto offset = line.GetOffset();
        fputc(2, stdout);
        for (uint32_t order = 0; order < 16; order++) {
          printf("%f", static_cast<double>(offset[order]));
          if (order < 15) {
            fputc(',', stdout);
          } else {
            fputc('\n', stdout);
          }
        }
        fputc(3, stdout);
        fflush(stdout);
      } break;
      case 0x0f:
        trace.PrintLog(true);
        break;
      case 0x1f:
        TestSelectMode();
        break;
      default:
        break;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
#endif  // APP_UNIT_TEST
