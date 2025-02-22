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
#include "NonVolatileData.h"
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

extern "C" void vAPP_TaskEntry() {
  Initialize();
  ShowBatteryVoltage();

  auto &ui = Ui::Instance();

  /* キャリブレーションデータを読み出し */
  if (!LineSensing::LineSensing::Instance().LoadCalibrationData()) {
    ui.Fatal();
  }

  /* スイッチから手が離れるまで待つ */
  ui.SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
  vTaskDelay(1000);
  ui.SetIndicator(0x00, 0xff);

  uint8_t mode = 0;
  auto &trace = Trace::Instance();
  /* モード選択 */
  while (true) {
    mode = SelectMode(0x1f, mode);
    switch (mode) {
      case 0x01: {
        /* 探索走行(非吸引で確実に走る速度、位置精度を向上させるために一応吸う) */
        /* TODO: 加速度を上げる */
        Trace::Parameter param = {
            Trace::Mode::kSearchRunning, /* モード */
            1,                           /* ログ周期 [ms] */
            1.2f,                        /* 探索上限速度・最短時初期速度 [m/s] */
            5.0f,                        /* 加速度 [m/ss] */
            0.0f,                        /* 最短時減速度 [m/ss] */
            {5.0f, 0.08f, 0.0f},         /* 並進PIDゲイン */
            {0.6f, 0.02f, 0.0f},         /* 旋回PIDゲイン */
            {7.0f, 0.0f, 0.01f},         /* ライン追従PIDゲイン */
            0.2f,                        /* ゴールマーカーから停止までの距離 [m] */
            2.0f,                        /* 吸引電圧 [V] */
        };
        trace.Run(param);
      } break;
      case 0x02: {
        /* 探索走行(吸引で確実に走る速度、最短走行が成功しない場合にタイムを縮めるために使用) */
        /* TODO: 加速度を上げる */
        Trace::Parameter param = {
            Trace::Mode::kSearchRunning, /* モード */
            10,                          /* ログ周期 [ms] */
            1.5f,                        /* 探索速度[m/s] */
            5.0f,                        /* 加速度 [m/ss] */
            0.0f,                        /* 最短時減速度 [m/ss] */
            {5.0f, 0.08f, 0.0f},         /* 並進PIDゲイン */
            {0.6f, 0.02f, 0.0f},         /* 旋回PIDゲイン */
            {7.0f, 0.0f, 0.01f},         /* ライン追従PIDゲイン */
            0.2f,                        /* ゴールマーカーから停止までの距離 [m] */
            2.0f,                        /* 吸引電圧 [V] */
        };
        trace.Run(param);
      } break;
      case 0x03: {
        /* 探索走行(吸引で大体走る速度、最短走行が成功しない場合にタイムを縮めるために使用・最短ゲイン調整用) */
        Trace::Parameter param = {
            Trace::Mode::kSearchRunning, /* モード */
            1,                           /* ログ周期 [ms] */
            2.0f,                        /* 探索速度[m/s] */
            10.0f,                       /* 加速度 [m/ss] */
            0.0f,                        /* 最短時減速度 [m/ss] */
            {5.0f, 0.08f, 0.0f},         /* 並進PIDゲイン */
            {0.8f, 0.02f, 0.0f},         /* 旋回PIDゲイン */
            {13.0f, 0.0f, 0.01f},        /* ライン追従PIDゲイン */
            0.2f,                        /* ゴールマーカーから停止までの距離 [m] */
            4.0f,                        /* 吸引電圧 [V] */
        };
        trace.Run(param);
      } break;
      case 0x04: {
        /* 最短走行1 */
        std::vector<float> minRadius = {
            /* 曲率マップ */
            0.2f, 0.4f, 0.65f, 1.5f, 2.0f, 5.0f,
        };
        std::vector<float> maxVelocity = {
            /* 速度マップ */
            1.0f, 2.0f, 2.2f, 3.0f, 3.2f, 3.5f,
        };
        Trace::Parameter param = {
            Trace::Mode::kFastRunning, /* モード */
            10,                        /* ログ周期 [ms] */
            2.0f,                      /* スタート時目標速度[m/s] */
            6.0f,                      /* 加速度 [m/ss] */
            6.0f,                      /* 最短時減速度 [m/ss] */
            {5.0f, 0.08f, 0.0f},       /* 並進PIDゲイン */
            {0.8f, 0.02f, 0.0f},       /* 旋回PIDゲイン */
            {4.5f, 0.0f, 0.005f},      /* ライン追従PIDゲイン */
            0.2f,                      /* ゴールマーカーから停止までの距離 [m] */
            3.5f,                      /* 吸引電圧 [V] */
        };
        trace.CalculateVelocityMap(minRadius, maxVelocity, param.maxVelocity, param.acceleration, param.deceleration);
        trace.Run(param);
      } break;
      case 0x05: {
        /* 最短走行2 */
        std::vector<float> minRadius = {
            /* 曲率マップ */
            0.2f, 0.4f, 0.65f, 1.5f, 2.0f, 5.0f,
        };
        std::vector<float> maxVelocity = {
            /* 速度マップ */
            1.5f, 2.0f, 2.5f, 3.0f, 3.5f, 4.0f,
        };
        Trace::Parameter param = {
            Trace::Mode::kFastRunning, /* モード */
            10,                        /* ログ周期 [ms] */
            2.0f,                      /* スタート時目標速度[m/s] */
            8.0f,                      /* 加速度 [m/ss] */
            8.0f,                      /* 最短時減速度 [m/ss] */
            {5.0f, 0.08f, 0.0f},       /* 並進PIDゲイン */
            {0.6f, 0.02f, 0.0f},       /* 旋回PIDゲイン */
            {9.0f, 0.0f, 0.01f},       /* ライン追従PIDゲイン */
            0.2f,                      /* ゴールマーカーから停止までの距離 [m] */
            3.5f,                      /* 吸引電圧 [V] */
        };
        trace.CalculateVelocityMap(minRadius, maxVelocity, param.maxVelocity, param.acceleration, param.deceleration);
        trace.Run(param);
      } break;
      case 0x06: {
        /* 最短走行3 */
        std::vector<float> minRadius = {
            /* 曲率マップ */
            0.2f, 0.4f, 0.65f, 1.5f, 2.0f, 5.0f,
        };
        std::vector<float> maxVelocity = {
            /* 速度マップ */
            1.8f, 2.3f, 2.8f, 3.3f, 3.8f, 4.3f,
        };
        Trace::Parameter param = {
            Trace::Mode::kFastRunning, /* モード */
            1,                         /* ログ周期 [ms] */
            2.0f,                      /* スタート時目標速度[m/s] */
            8.0f,                      /* 加速度 [m/ss] */
            8.0f,                      /* 最短時減速度 [m/ss] */
            {5.0f, 0.08f, 0.0f},       /* 並進PIDゲイン */
            {0.6f, 0.02f, 0.0f},       /* 旋回PIDゲイン */
            {11.0f, 0.0f, 0.01f},      /* ライン追従PIDゲイン */
            0.2f,                      /* ゴールマーカーから停止までの距離 [m] */
            4.0f,                      /* 吸引電圧 [V] */
        };
        trace.CalculateVelocityMap(minRadius, maxVelocity, param.maxVelocity, param.acceleration, param.deceleration);
        trace.Run(param);
      } break;
      case 0x07: {
        /* 最短走行4 */
        std::vector<float> minRadius = {
            /* 曲率マップ */
            0.2f, 0.4f, 0.5f, 0.65f, 1.5f, 2.0f, 5.0f,
        };
        std::vector<float> maxVelocity = {
            /* 速度マップ */
            2.0f, 2.2f, 3.0f, 3.5f, 4.0f, 4.5f, 5.0f,
        };
        Trace::Parameter param = {
            Trace::Mode::kFastRunning, /* モード */
            1,                         /* ログ周期 [ms] */
            2.0f,                      /* スタート時目標速度[m/s] */
            8.0f,                      /* 加速度 [m/ss] */
            8.0f,                      /* 最短時減速度 [m/ss] */
            {5.0f, 0.08f, 0.0f},       /* 並進PIDゲイン */
            {0.8f, 0.02f, 0.0f},       /* 旋回PIDゲイン */
            {13.0f, 0.0f, 0.01f},      /* ライン追従PIDゲイン */
            0.2f,                      /* ゴールマーカーから停止までの距離 [m] */
            4.0f,                      /* 吸引電圧 [V] */
        };
        trace.CalculateVelocityMap(minRadius, maxVelocity, param.maxVelocity, param.acceleration, param.deceleration);
        trace.Run(param);
      } break;
      case 0x08: {
        /* ラインセンサーのキャリブレーション */
        if (!LineSensing::LineSensing::Instance().StoreCalibrationData(2000)) {
          ui.Warn();
        }
        ui.SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
      } break;
      case 0x09: {
        /* 探索データをFRAMから復元 */
        trace.LoadSearchRunningPoints();
        ui.SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
      } break;
      case 0x0a:
        trace.PrintLog();
        break;
      case 0x0b:
        trace.PrintSearchRunningPoints();
        break;
      case 0x0c: {
        trace.PrintVelocityTable();
      } break;
      case 0x10: {
        /* (これを本番で使うことはない)最短走行調整用 */
        Trace::Parameter param = {
            Trace::Mode::kFastRunning, /* モード */
            10,                        /* ログ周期 [ms] */
            1.0f,                      /* スタート時目標速度[m/s] */
            6.0f,                      /* 加速度 [m/ss] */
            6.0f,                      /* 最短時減速度 [m/ss] */
            {5.0f, 0.08f, 0.0f},       /* 並進PIDゲイン */
            {0.6f, 0.02f, 0.0f},       /* 旋回PIDゲイン */
            {7.0f, 0.0f, 0.01f},       /* ライン追従PIDゲイン */
            0.2f,                      /* ゴールマーカーから停止までの距離 [m] */
            0.0f,                      /* 吸引電圧 [V] */
        };
        std::vector<float> minRadius = {
            0.2f, 0.4f, 0.6f, 0.8f, 1.0f,
        };
        std::vector<float> maxVelocity = {
            1.0f, 1.25f, 1.5f, 1.75f, 2.0f,
        };
        trace.CalculateVelocityMap(minRadius, maxVelocity, param.maxVelocity, param.acceleration, param.acceleration);
        trace.Run(param);
      } break;
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
