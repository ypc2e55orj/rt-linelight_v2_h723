#include "Test.h"

/* Project */
#include "Com.h"
#include "Config.h"
#include "Data/Pid.h"
#include "Fram.h"
#include "LineSensing/LineSensing.h"
#include "Mode.h"
#include "MotionPlaning/MotionPlaning.h"
#include "MotionPlaning/Suction.h"
#include "MotionPlaning/VelocityGenerator.h"
#include "MotionSensing/Encoder.h"
#include "MotionSensing/MotionSensing.h"
#include "PowerMonitoring/PowerMonitoring.h"
#include "Ui.h"

/* C++ */
#include <cstdio>
#include <cstring>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdouble-promotion"

/* 車速測定テスト */
static void TestVelocityMeasure() {
  MotionSensing::MotionSensing::Instance().NotifyStart();
  auto &ui = Ui::Instance();
  auto &odometry = MotionSensing::MotionSensing::Instance().Odometry();
  auto xLastWakeTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    if (ui.WaitPress(0) >= kButtonShortPressThreshold) {
      ui.SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
      break;
    }
    auto acc = odometry.GetAcceleration();
    auto vel = odometry.GetVelocity();
    auto dis = odometry.GetDisplacement();

    printf("acc: %f, %f\r\n", acc.trans, acc.rot);
    printf("vel: %f, %f\r\n", vel.trans, vel.rot);
    printf("disp: %f, %f\r\n", dis.trans, dis.rot);
  }
  MotionSensing::MotionSensing::Instance().NotifyStop();
}
/* 宴会芸テスト */
static void TestEnkaigei() {
  auto &ui = Ui::Instance();
  auto &servo = MotionPlaning::MotionPlaning::Instance().Servo();
  servo.SetGain({0.5f, 0.00f, 0.0f}, {0.3f, 0.05f, 0.0f});

  /* IMUキャリブレーション */
  if (!MotionSensing::MotionSensing::Instance().CalibrateImu(1000)) {
    ui.Warn();
    return;
  }
  MotionPlaning::MotionPlaning::Instance().NotifyStart();
  MotionSensing::MotionSensing::Instance().NotifyStart();
  auto xLastWakeTime = xTaskGetTickCount();
  uint32_t count = 0;
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    if (ui.WaitPress(0) >= kButtonShortPressThreshold) {
      ui.SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
      break;
    }
    if (++count >= 10) {
      count = 0;
      auto tick = HAL_GetTick();
      auto ff = servo.GetFeedForwardAmount();
      auto fb = servo.GetFeedBackAmount();
      auto v = servo.GetMotorVoltage();
      printf(">t:%ld\n>ffr:%f\n>ffl:%f\n>fbv:%f\n>fbo:%f\n>vr:%f\n>vl:%f\n", tick, ff[0], ff[1], fb[0], fb[1], v[0],
             v[1]);
    }
  }
  MotionPlaning::MotionPlaning::Instance().NotifyStop();
  MotionSensing::MotionSensing::Instance().NotifyStop();
}
/* 直線動作 */
static void TestStraight() {
  struct TestStraightLog {
    uint32_t time;
    float targetVelocity;
    float measuredVelocity;
    float distance;
    float motorVoltageRight;
    float motorVoltageLeft;
    float motorCurrentRight;
    float motorCurrentLeft;
  };
  TestStraightLog testLog = {};
  uint32_t logAddr = 0;
  uint32_t t = 0;
  SlopeVelocityGenerator generator;
  SlopeVelocityGenerator::Profile profile = {0.0f, 0.5f, 0.0f, 1.0f, -1.0f, 1.0f};
  auto &odometry = MotionSensing::MotionSensing::Instance().Odometry();
  auto &servo = MotionPlaning::MotionPlaning::Instance().Servo();
  auto &power = PowerMonitoring::PowerMonitoring::Instance().Power();
  auto &fram = Fram::Instance();
  auto &ui = Ui::Instance();

  /* 時刻における目標速度を生成 */
  generator.Generate(profile);
  printf("Total: %ld ms\r\n", generator.GetTotalTime());

  servo.SetGain({5.0f, 0.01f, 0.0f}, {0.3f, 0.05f, 0.0f});
  /* IMUキャリブレーション */
  if (!MotionSensing::MotionSensing::Instance().CalibrateImu(1000)) {
    ui.Warn();
    return;
  }
  MotionPlaning::MotionPlaning::Instance().NotifyStart();
  MotionSensing::MotionSensing::Instance().NotifyStart();

  auto xLastWakeTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    float targetVelocity = generator.GetVelocity(++t);
    servo.SetTarget(targetVelocity, 0.0f);
    auto vel = odometry.GetVelocity();
    auto dis = odometry.GetDisplacement();
    auto dut = servo.GetMotorDuty();
    auto cur = power.GetMotorCurrent();
    testLog.time = t;
    testLog.targetVelocity = targetVelocity;
    testLog.measuredVelocity = vel.trans;
    testLog.distance = dis.trans;
    testLog.motorVoltageRight = dut[0];
    testLog.motorVoltageLeft = dut[1];
    testLog.motorCurrentRight = cur[0];
    testLog.motorCurrentLeft = cur[1];
    fram.Write(logAddr, &testLog, sizeof(testLog));
    logAddr += sizeof(testLog);
    if (generator.GetTotalTime() < t) {
      break;
    }
  }
  MotionPlaning::MotionPlaning::Instance().NotifyStop();
  MotionSensing::MotionSensing::Instance().NotifyStop();
  ui.SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
  ui.WaitPress();
  ui.SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
  fputc(2, stdout);
  printf("Col1, Col2, Col3, Col4, Col5\n");
  for (uint32_t addr = 0; addr < logAddr; addr += sizeof(testLog)) {
    fram.Read(addr, &testLog, sizeof(testLog));
    fprintf(stdout, "%lx, %ld, %f, %f, %f, %f, %f, %f, %f\n", addr, testLog.time, testLog.targetVelocity,
            testLog.measuredVelocity, testLog.distance, testLog.motorVoltageRight, testLog.motorVoltageLeft,
            testLog.motorCurrentRight, testLog.motorCurrentLeft);
  }
  fputc(3, stdout);
  fflush(stdout);
}
/* 旋回動作 */
static void TestTurn() {
  struct TestTurnLog {
    uint32_t time;
    float targetVelocity;
    float measuredVelocity;
    float distance;
    float motorVoltageRight;
    float motorVoltageLeft;
    float motorCurrentRight;
    float motorCurrentLeft;
  };
  TestTurnLog testLog = {};
  uint32_t logAddr = 0;
  uint32_t t = 0;
  SlopeVelocityGenerator generator;
  SlopeVelocityGenerator::Profile profile = {0.0f,
                                             static_cast<float>(M_PI) / 2.0f,
                                             0.0f,
                                             static_cast<float>(M_PI),
                                             -1.0f * static_cast<float>(M_PI),
                                             static_cast<float>(M_PI) * 2.0f};
  auto &odometry = MotionSensing::MotionSensing::Instance().Odometry();
  auto &servo = MotionPlaning::MotionPlaning::Instance().Servo();
  auto &power = PowerMonitoring::PowerMonitoring::Instance().Power();
  auto &fram = Fram::Instance();
  auto &ui = Ui::Instance();

  /* 時刻における目標速度を生成 */
  generator.Generate(profile);
  printf("Total: %ld ms\r\n", generator.GetTotalTime());

  servo.SetGain({0.0f, 0.0f, 0.0f}, {0.3f, 0.05f, 0.0f});
  /* IMUキャリブレーション */
  if (!MotionSensing::MotionSensing::Instance().CalibrateImu(1000)) {
    ui.Warn();
    return;
  }
  MotionPlaning::MotionPlaning::Instance().NotifyStart();
  MotionSensing::MotionSensing::Instance().NotifyStart();

  auto xLastWakeTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    float targetVelocity = generator.GetVelocity(++t);
    servo.SetTarget(0.0f, targetVelocity);
    auto vel = odometry.GetVelocity();
    auto dis = odometry.GetDisplacement();
    auto dut = servo.GetMotorDuty();
    auto cur = power.GetMotorCurrent();
    testLog.time = t;
    testLog.targetVelocity = targetVelocity;
    testLog.measuredVelocity = vel.rot;
    testLog.distance = dis.rot;
    testLog.motorVoltageRight = dut[0];
    testLog.motorVoltageLeft = dut[1];
    testLog.motorCurrentRight = cur[0];
    testLog.motorCurrentLeft = cur[1];
    fram.Write(logAddr, &testLog, sizeof(testLog));
    logAddr += sizeof(testLog);
    if (generator.GetTotalTime() < t) {
      break;
    }
  }
  MotionPlaning::MotionPlaning::Instance().NotifyStop();
  MotionSensing::MotionSensing::Instance().NotifyStop();
  ui.SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
  ui.WaitPress();
  ui.SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
  fputc(2, stdout);
  printf("Col1, Col2, Col3, Col4, Col5\n");
  for (uint32_t addr = 0; addr < logAddr; addr += sizeof(testLog)) {
    fram.Read(addr, &testLog, sizeof(testLog));
    fprintf(stdout, "%lx, %ld, %f, %f, %f, %f, %f, %f, %f\n", addr, testLog.time, testLog.targetVelocity,
            testLog.measuredVelocity, testLog.distance, testLog.motorVoltageRight, testLog.motorVoltageLeft,
            testLog.motorCurrentRight, testLog.motorCurrentLeft);
  }
  fputc(3, stdout);
  fflush(stdout);
}
/* ラインセンサーテスト */
static void TestLineMark() {
  auto &ui = Ui::Instance();
  auto &line = LineSensing::LineSensing::Instance().Line();
  auto &marker = LineSensing::LineSensing::Instance().Marker();

  LineSensing::LineSensing::Instance().NotifyStart();
  auto xLastWakeTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    if (ui.WaitPress(0) >= kButtonShortPressThreshold) {
      ui.SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
      break;
    }
    auto ea = line.GetError() * 180.0f / static_cast<float>(M_PI);
    auto dn = line.GetDetectNum();
    auto mc = marker.GetCount();
    printf("%ld, %f, %d, %ld, %ld\r\n", HAL_GetTick(), ea, dn, mc[0], mc[1]);
  }
  LineSensing::LineSensing::Instance().NotifyStop();
}
/* ラインセンサー生値テスト */
static void TestLineRaw() {
  auto &ui = Ui::Instance();
  auto &line = LineSensing::LineSensing::Instance().Line();
  LineSensing::LineImpl::Raw raw{};
  LineSensing::LineSensing::Instance().NotifyStart();
  auto xLastWakeTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    if (ui.WaitPress(0) >= kButtonShortPressThreshold) {
      ui.SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
      break;
    }
    line.GetRaw(raw);
    for (uint32_t order = 0; order < 16; order++) {
      fprintf(stdout, "%05d", raw[order]);
      if (order != 15) {
        fprintf(stdout, ",");
      } else {
        fprintf(stdout, "\r\n");
      }
    }
  }
  LineSensing::LineSensing::Instance().NotifyStop();
}
/* タスクリストを表示 */
static void TestTaskList() {
  auto &ui = Ui::Instance();
  auto xLastWakeTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    if (ui.WaitPress(0) >= kButtonShortPressThreshold) {
      ui.SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
      break;
    }

    /* Task Info */
    {
      char taskInfoBuffer[512] = {0};
      uint32_t ulTotalRunTime = HAL_GetTick();
      uint32_t ulTotalHour = ulTotalRunTime / 3600000;
      uint32_t ulTotalMin = (ulTotalRunTime % 3600000) / 60000;
      uint32_t ulTotalSec = (ulTotalRunTime % 60000) / 1000;
      uint32_t ulTotalMsec = ulTotalRunTime % 1000;
      vTaskList(taskInfoBuffer);
      printf("----- Task Info (%02ld:%02ld:%02ld.%03ld) -----\r\n%s\r\n", ulTotalHour, ulTotalMin, ulTotalSec,
             ulTotalMsec, taskInfoBuffer);
    }
  }
}
/* ラインセンサーの値を収集 */
static void TestCollectLineSensor() {
  auto &ui = Ui::Instance();
  fputc(2, stdout);
  fflush(stdout);
  auto &line = LineSensing::LineSensing::Instance().Line();
  LineSensing::LineSensing::Instance().NotifyStart();
  LineSensing::LineImpl::Raw raw{};
  auto xLastWakeTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    uint32_t pressTime = ui.WaitPress();
    if (pressTime >= kButtonLongPressThreshold) {
      ui.SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
      break;
    } else if (pressTime >= kButtonShortPressThreshold) {
      ui.SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
      line.GetRaw(raw);
      for (uint32_t order = 0; order < 16; order++) {
        fprintf(stdout, "%05d", raw[order]);
        if (order != 15) {
          fprintf(stdout, ",");
        }
      }
      fprintf(stdout, "\n");
    }
  }
  fputc(3, stdout);
  fflush(stdout);
  LineSensing::LineSensing::Instance().NotifyStop();
}
/* FRAM ダンプ */
static void TestFramDumpAll() {
  auto &fram = Fram::Instance();
  auto &ui = Ui::Instance();
  uint32_t address = 0;
  printf("%6s", "");
  for (int i = 0; i < 4; i++) {
    printf(" %-7x ", i * 4);
  }
  printf("\r\n");

  auto xLastWakeTime = xTaskGetTickCount();
  while (address < fram.kMaxAddress) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    if (ui.WaitPress(0)) {
      ui.SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
      break;
    }
    printf("%03lx: ", address);
    for (int i = 0; i < 4; i++) {
      uint8_t data[4] = {0};
      fram.Read(address, data, 4);
      printf("%02x%02x%02x%02x ", data[0], data[1], data[2], data[3]);
      address += 4;
    }
    printf("\r\n");
  }
}
/* 吸引ファンテスト */
static void TestSuctionFan() {
  auto &ui = Ui::Instance();
  auto &power = PowerMonitoring::PowerMonitoring::Instance().Power();
  auto &suction = MotionPlaning::Suction::Instance();

  suction.Enable();
  auto xLastWakeTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    if (ui.WaitPress(0)) {
      ui.SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
      break;
    }
    suction.SetDuty(4.2f / power.GetBatteryVoltage());
  }
  suction.Disable();
}
/* エンコーダーテスト */
static void TestEncoder() {
  auto &ui = Ui::Instance();
  auto &encoder = MotionSensing::Encoder::Instance();
  MotionSensing::MotionSensing::Instance().NotifyStart();
  auto xLastWakeTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    if (ui.WaitPress(0)) {
      ui.SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
      break;
    }
    auto enc = encoder.GetCount();
    printf("%d, %d\r\n", enc[0], enc[1]);
  }
  MotionSensing::MotionSensing::Instance().NotifyStop();
}
/* ライン追従テスト */
static void TestLineFollow() {
  auto &ui = Ui::Instance();
  auto &line = LineSensing::LineSensing::Instance().Line();
  auto &odom = MotionSensing::MotionSensing::Instance().Odometry();
  auto &servo = MotionPlaning::MotionPlaning::Instance().Servo();
  uint32_t count = 0;
  Pid pid{};
  pid.Reset({3.5f, 0.0f, 0.01f});
  servo.SetGain({0.0f, 0.00f, 0.0f}, {0.3f, 0.05f, 0.0f});
  /* IMUキャリブレーション */
  if (!MotionSensing::MotionSensing::Instance().CalibrateImu(1000)) {
    ui.Warn();
    return;
  }
  MotionPlaning::MotionPlaning::Instance().NotifyStart();
  MotionSensing::MotionSensing::Instance().NotifyStart();
  LineSensing::LineSensing::Instance().NotifyStart();
  auto xLastWakeTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    if (ui.WaitPress(0) >= kButtonShortPressThreshold) {
      ui.SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
      break;
    }

    auto error = line.GetError();
    auto angVelo = pid.Update(0, error, kPeriodicNotifyInterval);
    servo.SetTarget(0, angVelo);
    if (++count >= 100) {
      count = 0;
      printf(">e:%f\n>c:%f\n>m:%f\n", static_cast<double>(error), static_cast<double>(angVelo),
             static_cast<double>(odom.GetVelocity().rot));
    }
  }
  LineSensing::LineSensing::Instance().NotifyStop();
  MotionSensing::MotionSensing::Instance().NotifyStop();
  MotionPlaning::MotionPlaning::Instance().NotifyStop();
}
/* テスト用モードセレクト */
void TestSelectMode() {
  /* モード選択 */
  uint8_t mode = 0;
  while (true) {
    mode = SelectMode(0x0f, mode);
    switch (mode) {
      case 0x01:
        TestLineMark();
        break;
      case 0x2:
        TestTurn();
        break;
      case 0x03:
        TestStraight();
        break;
      case 0x04:
        TestVelocityMeasure();
        break;
      case 0x05:
        TestEnkaigei();
        break;
      case 0x06:
        TestFramDumpAll();
        break;
      case 0x07:
        printf("Fram::Clear() started\r\n");
        Fram::Instance().Clear();
        printf("Fram::Clear() complated\r\n");
        break;
      case 0x08:
        TestTaskList();
        break;
      case 0x09:
        TestCollectLineSensor();
        break;
      case 0x0a:
        TestSuctionFan();
        break;
      case 0x0b:
        TestLineRaw();
        break;
      case 0x0c:
        TestEncoder();
        break;
      case 0x0d:
        TestLineFollow();
        break;
      case 0x0f:
        return; /* メインに戻る */
      default:
        break;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

#pragma GCC diagnostic pop
