#include "Trace.h"

/* Project */
#include "Config.h"
#include "Periodic.h"

/* FreeRTOS */
#include <FreeRTOS.h>

/* C++ */
#include <cstdio>
#include <cstring>

using LineState = LineSensing::LineImpl::State;
using Landmark = MotionPlaning::PositionCorrector::Landmark;

/* コンストラクタ */
Trace::Trace() : velocityMap_(radiusExplorer_) {
  ms_ = &MotionSensing::MotionSensing::Instance();
  ls_ = &LineSensing::LineSensing::Instance();
  mp_ = &MotionPlaning::MotionPlaning::Instance();
  ui_ = &Ui::Instance();
  servo_ = &MotionPlaning::MotionPlaning::Instance().Servo();
  suction_ = &MotionPlaning::Suction::Instance();
  odometry_ = &MotionSensing::MotionSensing::Instance().Odometry();
  line_ = &LineSensing::LineSensing::Instance().Line();
  marker_ = &LineSensing::LineSensing::Instance().Marker();
  power_ = &PowerMonitoring::PowerMonitoring::Instance().Power();
  fram_ = &Fram::Instance();
}

/**
 * MARK: Run
 */
void Trace::Run(const Parameter &param) {
  /* リセット */
  state_ = kStateResetting;
  resetCount_ = 0;
  isEmergency_ = false;

  if (param.mode == Mode::kExploreRunning) {
    /* 既に探索済みの場合は警告 */
    if (radiusExplorer_.IsExplored()) {
      ui_->Warn();
      if (!Confirmed()) {
        return;
      }
    }
    radiusExplorer_.Reset();
  } else if (param.mode == Mode::kFastRunning) {
    /* 未探索か速度テーブルが未計算 */
    if (!velocityMap_.IsGenerated()) {
      ui_->Warn();
      return;
    }
    velocityMap_.ResetIndex();
  }

  /* パラメータを設定 */
  param_ = param;
  lineErrorPid_.Reset(param_.lineErrorGain);
  servo_->SetGain(param_.linearGain, param_.angularGain);

  /* 手が離れるまで待つ */
  vTaskDelay(pdMS_TO_TICKS(1000));

  /* IMUキャリブレーション */
  if (!ms_->CalibrateImu(1000)) {
    ui_->Warn();
    return;
  }

  /* 各タスクを開始 */
  ms_->NotifyStart();
  ls_->NotifyStart();
  mp_->NotifyStart();
  while (state_ != kStateStoped) {
    if (!Periodic::WaitPeriodicNotify()) {
      state_ = kStateEmergency;
      break;
    }
    UpdateState();
    UpdateMotion();
    UpdateLog();
  }
  OnStopped();
  mp_->NotifyStop();
  ls_->NotifyStop();
  ms_->NotifyStop();
  if (isEmergency_) {
    ui_->Warn();
  }
  vTaskDelay(pdMS_TO_TICKS(100));
}

/* 操作者に確認 */
bool Trace::Confirmed() {
  uint32_t pressTime = ui_->WaitPress();
  if (pressTime >= kButtonLongPressThreshold) {
    ui_->SetBuzzer(kBuzzerFrequency, kBuzzerCancelDuration);
    return false;
  }
  ui_->SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
  vTaskDelay(pdMS_TO_TICKS(1000));
  return true;
}

/**
 * MARK: State
 */
void Trace::UpdateState() {
  /* 状態の変更と状態コールバックのみ */
  if (CheckEmergency()) {
    state_ = kStateEmergency;
  }
  switch (state_) {
    case kStateResetting:
      /* リセット */
      OnResetting();
      if (++resetCount_ >= 1000) {
        state_ = kStateStartWaiting;
      }
      break;
    case kStateStartWaiting:
      /* スタートマーカーを待つ */
      OnStartWaiting();
      if (marker_->IsStarted()) {
        state_ = kStateStarted;
      }
      break;
    case kStateStarted:
      /* スタートマーカー通過 */
      OnStarted();
      state_ = kStateGoalWaiting;
      break;
    case kStateGoalWaiting:
      /* ゴールマーカーを待つ */
      OnGoalWaiting();
      if (marker_->IsGoaled()) {
        state_ = kStateGoaled;
      }
      break;
    case kStateEmergency:
      /* 緊急停止 */
      OnEmergency();
      state_ = kStateStopWaiting;
      break;
    case kStateGoaled:
      /* ゴールマーカー通過 */
      OnGoaled();
      state_ = kStateStopWaiting;
      break;
    case kStateStopWaiting:
      /* 減速中 */
      OnStopWaiting();
      if (velocity_ < 0.01f) {
        state_ = kStateStoped;
      }
      break;
    case kStateStoped:
      /* 停止 */
      /* ここでwhileを抜ける */
      break;
  }
}

/* リセット */
void Trace::OnResetting() {
  /* 走行制御 */
  suction_->Enable();
  limitVelocity_ = 0.0f;
  velocity_ = 0.0f;
  acceleration_ = 0.0f;
  /* angularVelocity_ = 0.0f; */ /* 吸引ファンで動いてしまうため */

  /* ログ */
  logFrequencyCount_ = 0;
  log_ = {};
  logAddress_ = 0;
  logEnabled_ = false;
}
/* スタートマーカーを待つ */
void Trace::OnStartWaiting() {
  /* 走行制御 */
  limitVelocity_ = param_.limitVelocity;
  acceleration_ = param_.acceleration;
}
/* スタートマーカー通過 */
void Trace::OnStarted() {
  ui_->SetBuzzer(kBuzzerFrequency, 50);
  ui_->SetIndicator(0x60, 0x60);

  /* 走行制御 */
  odometry_->Reset();

  /* ログ */
  logEnabled_ = true;
  logStartTime_ = HAL_GetTick();
}
/* ゴールマーカーを待つ */
void Trace::OnGoalWaiting() {
  /* 位置補正 */
  auto deltaDistance = odometry_->GetDisplacementTranslateDelta();
  if (param_.mode == Mode::kExploreRunning) {
    /* 曲率探索 */
    /* 距離と距離単位あたりの角度を保存 */
    auto totalDistance = odometry_->GetDisplacement().trans;
    radiusExplorer_.Update(deltaDistance, odometry_->GetVelocity().rot);

    /* 位置補正 */
    /* 記録には補正されていない値を使用すること */
    if (marker_->IsCurvature()) { /* とりあえず曲率マーカーを優先 どっちの方が正確？ */
      positionCorrector_.Store(Landmark::kCurvatureMark, totalDistance);
    } else if (line_->IsCrossPassed()) {
      positionCorrector_.Store(Landmark::kCrossLine, totalDistance);
    }
  } else if (param_.mode == Mode::kFastRunning) {
    /* 走行制御 */
    /* 最短時は生成したテーブルから速度を索引 */
    velocityMap_.UpdateDistance(deltaDistance);
    auto totalDistance = velocityMap_.GetTotalDistance();
    { /* 位置補正 */
      float correctedDistance = 0.0f;
      if (marker_->IsCurvature()) {
        correctedDistance = positionCorrector_.Correct(Landmark::kCurvatureMark, totalDistance);
        velocityMap_.CorrectDistance(correctedDistance);
      } else if (line_->IsCrossPassed()) {
        correctedDistance = positionCorrector_.Correct(Landmark::kCrossLine, totalDistance);
        velocityMap_.CorrectDistance(correctedDistance);
      }
    }
    velocityMap_.UpdateIndex();
    auto now = velocityMap_.GetVelocity();
    auto next = velocityMap_.GetNextVelocity();
    if (next < now) {
      limitVelocity_ = std::abs(now);
      acceleration_ = -1.0f * param_.acceleration;
    } else { /* next > now */
      limitVelocity_ = std::abs(next);
      acceleration_ = param_.acceleration;
    }
  }
}
/* 緊急状態かどうか */
bool Trace::CheckEmergency() {
  if (ui_->WaitPress(0)) { /* ボタンが押されている */
    return true;
  } else if (line_->IsNone()) { /* ラインが見えない */
    return true;
  } else if (power_->GetBatteryErrorTime() > kBatteryErrorLimit) { /* バッテリーエラー */
    return true;
  }
  return false;
}
/* 緊急停止 */
void Trace::OnEmergency() {
  isEmergency_ = true;

  /* 走行制御 */
  acceleration_ = CalculateDeceleration(velocity_, param_.stopDistance);
}
/* ゴールマーカー通過 */
void Trace::OnGoaled() {
  ui_->SetBuzzer(kBuzzerFrequency, 50);
  ui_->SetIndicator(0x00, 0x60);

  /* 曲率探索 */
  if (param_.mode == Mode::kExploreRunning) {
    radiusExplorer_.Explored();
    velocityMap_.ResetGenerated();
  }

  /* 走行制御 */
  acceleration_ = CalculateDeceleration(velocity_, param_.stopDistance);
}
/* 減速中 */
void Trace::OnStopWaiting() {}
/* 停止 */
void Trace::OnStopped() {
  /* 走行制御 */
  limitVelocity_ = 0.0f;
  angularVelocity_ = 0.0f;
  suction_->Disable();
}

/* 走行制御を更新 */
void Trace::UpdateMotion() {
  /* 必要があれば吸引ファンを回す */
  if (param_.suctionVoltage != 0.0f) {
    suction_->SetDuty(param_.suctionVoltage / power_->GetBatteryVoltage());
  }
  /* 設定された制限速度を元に加減速した速度を計算 */
  velocity_ += acceleration_ * kPeriodicNotifyInterval;
  if (std::abs(velocity_) > limitVelocity_) {
    velocity_ = std::copysign(limitVelocity_, velocity_);
  }
  /* ライン追従角速度を計算 */
  angularVelocity_ = lineErrorPid_.Update(0, line_->GetErrorAngle(), kPeriodicNotifyInterval);
  /* 設定 */
  servo_->SetTarget(velocity_, angularVelocity_);
}
/* 現在の速度から指定距離で停止する加速度を計算 */
float Trace::CalculateDeceleration(float velocity, float distance) {
  return -1.0f * std::pow(velocity, 2.0f) / (2.0f * distance);
}

/* ログを更新 */
void Trace::UpdateLog() {
  bool isWrite = false;

  /* 定期書き込み */
  if (logEnabled_ && ++logFrequencyCount_ >= param_.logInterval) {
    logFrequencyCount_ = 0;
    isWrite = true;
  }
  if (isWrite && logAddress_ + sizeof(Log) < Fram::kMaxAddress) {
    auto et = static_cast<float>(velocityMap_.GetIndex()) * 0.01f;
    auto vel = odometry_->GetVelocity();
    auto dis = odometry_->GetDisplacement();
    auto vol = servo_->GetMotorVoltage();
    auto cur = power_->GetMotorCurrent();
    auto pos = odometry_->GetPose();
    auto ms = marker_->GetState();
    log_.time = HAL_GetTick() - logStartTime_;                      /* 00 Time */
    log_.line = line_->GetState();                                  /* 01 Line State */
    log_.commandVelocity = velocity_;                               /* 02 Command Velocity */
    log_.estimateVelocity = vel.trans;                              /* 03 Estimate Velocity */
    log_.expectTranslate = et;                                      /* 04 Expect Translate */
    log_.estimateTranslate = dis.trans;                             /* 05 Estimate Translate */
    log_.correctedTranslate = velocityMap_.GetTotalDistance();      /* 06 Corrected Translate */
    log_.errorAngle = line_->GetErrorAngle();                       /* 07 Error Angle */
    log_.commandAngularVelocity = lineErrorPid_.Get();              /* 08 Command Angular Velocity */
    log_.commandAngularVelocityP = lineErrorPid_.GetProportional(); /* 09 Command Angular Velocity (P) */
    log_.commandAngularVelocityI = lineErrorPid_.GetIntegral();     /* 10 Command Angular Velocity (I) */
    log_.commandAngularVelocityD = lineErrorPid_.GetDerivative();   /* 11 Command Angular Velocity (D) */
    log_.estimateAngularVelocity = vel.rot;                         /* 12 Estimate Angular Velocity */
    log_.estimateRotate = dis.rot;                                  /* 13 Estimate Rotate */
    log_.batteryVoltage = power_->GetBatteryVoltage();              /* 14 Battery Voltage */
    log_.motorVoltageRight = vol[0];                                /* 15 Motor Voltage Right */
    log_.motorVoltageLeft = vol[1];                                 /* 16 Motor Voltage Left */
    log_.motorCurrentRight = cur[0];                                /* 17 Motor Current Right */
    log_.motorCurrentLeft = cur[1];                                 /* 18 Motor Current Left */
    log_.x = pos.x;                                                 /* 19 X */
    log_.y = pos.y;                                                 /* 20 Y */
    log_.theta = pos.theta;                                         /* 21 Theta */
    log_.markerRight = ms[0];                                       /* 22 Marker Right State */
    log_.markerLeft = ms[1];                                        /* 23 Marker Left State */
    fram_->Write(logAddress_, &log_, sizeof(Log));
    logAddress_ += sizeof(Log);
  }
}

/* 速度マップを計算 */
void Trace::CalculateVelocityMap(std::vector<RadiusVelocityLimit> &limits, float startVelocity, float acceleration,
                                 float deceleration, uint32_t shift) {
  if (velocityMap_.IsGenerated()) {
    ui_->Warn();
    if (!Confirmed()) {
      return;
    }
  }
  if (!velocityMap_.Generate(limits, startVelocity, acceleration, deceleration, shift)) {
    ui_->Warn();
    return;
  }
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdouble-promotion"
/**
 * MARK: PrintLog
 * この中はログ出力関係のみにすること
 */

/* ログを出力 */
void Trace::PrintLog(bool force) {
  uint32_t prevAddress = 0;
  if (!force && logAddress_ == 0) {
    vTaskDelay(pdMS_TO_TICKS(100));
    ui_->Warn();
    return;
  }
  if (force) {
    prevAddress = logAddress_;
    logAddress_ = Fram::kMaxAddress;
  }

  fputc(2, stdout);
  fprintf(stdout,
          "Address, "
          "Time, "
          "Line State, "
          "Command Velocity, "
          "Estimate Velocity, "
          "Expect Translate, "
          "Estimate Translate, "
          "Corrected Translate, "
          "Error Angle, "
          "Command Angular Velocity, "
          "Command Angular Velocity (P), "
          "Command Angular Velocity (I), "
          "Command Angular Velocity (D), "
          "Estimate Angular Velocity, "
          "Estimate Rotate, "
          "Battery Voltage, "
          "Motor Voltage Right, "
          "Motor Voltage Left, "
          "Motor Current Right, "
          "Motor Current Left, "
          "X, "
          "Y, "
          "Theta, "
          "Marker Right State, "
          "Marker Left State"
          "\n" /* */
  );
  auto xLastWakeTime = xTaskGetTickCount();
  for (uint32_t i = 0; i < logAddress_; i += sizeof(Log)) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
    fram_->Read(i, &log_, sizeof(Log));

    fprintf(stdout,
            "%lx, "                             /* xx Address */
            "%lu, "                             /* 00 Time */
            "%d, "                              /* 01 Line State */
            "%f, "                              /* 02 Command Velocity */
            "%f, "                              /* 03 Estimate Velocity */
            "%f, "                              /* 04 Expect Translate */
            "%f, "                              /* 05 Estimate Translate */
            "%f, "                              /* 06 Corrected Translate */
            "%f, "                              /* 07 Error Angle */
            "%f, "                              /* 08 Command Angular Velocity */
            "%f, "                              /* 09 Command Angular Velocity (P) */
            "%f, "                              /* 10 Command Angular Velocity (I) */
            "%f, "                              /* 11 Command Angular Velocity (D) */
            "%f, "                              /* 12 Estimate Rotate Velocity */
            "%f, "                              /* 13 Estimate Rotate */
            "%f, "                              /* 14 Battery Voltage */
            "%f, "                              /* 15 Motor Voltage Right */
            "%f, "                              /* 16 Motor Voltage Left */
            "%f, "                              /* 17 Motor Current Right */
            "%f, "                              /* 18 Motor Current Left */
            "%f, "                              /* 19 X */
            "%f, "                              /* 20 Y */
            "%f, "                              /* 21 Theta */
            "%d, "                              /* 22 Marker Right State */
            "%d"                                /* 23 Marker Left State */
            "\n",                               /* */
            i,                                  /* xx Address */
            log_.time,                          /* 00 Time */
            static_cast<int>(log_.line),        /* 01 Line State */
            log_.commandVelocity,               /* 02 Command Velocity */
            log_.estimateVelocity,              /* 03 Estimate Velocity */
            log_.expectTranslate,               /* 04 Expect Translate */
            log_.estimateTranslate,             /* 05 Estimate Translate */
            log_.correctedTranslate,            /* 06 Corrected Translate */
            log_.errorAngle,                    /* 07 Error Angle */
            log_.commandAngularVelocity,        /* 08 Command Angular Velocity */
            log_.commandAngularVelocityP,       /* 09 Command Angular Velocity (P) */
            log_.commandAngularVelocityI,       /* 10 Command Angular Velocity (I) */
            log_.commandAngularVelocityD,       /* 11 Command Angular Velocity (D) */
            log_.estimateAngularVelocity,       /* 12 Estimate Angular Velocity */
            log_.estimateRotate,                /* 13 Estimate Rotate */
            log_.batteryVoltage,                /* 14 Battery Voltage */
            log_.motorVoltageRight,             /* 15 Motor Voltage Right */
            log_.motorVoltageLeft,              /* 16 Motor Voltage Left */
            log_.motorCurrentRight,             /* 17 Motor Current Right */
            log_.motorCurrentLeft,              /* 18 Motor Current Left */
            log_.x,                             /* 19 X */
            log_.y,                             /* 20 Y */
            log_.theta,                         /* 21 Theta */
            static_cast<int>(log_.markerRight), /* 22 Marker Right State */
            static_cast<int>(log_.markerLeft)   /* 23 Marker Left State */
    );
  }
  fputc(3, stdout);
  fflush(stdout);
  if (force) {
    logAddress_ = prevAddress;
  }

  ui_->SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
}

/* 加減速生成用のログを出力 */
void Trace::PrintRadiusExplorerLog() {
  auto &veloMap = radiusExplorer_.Get();
  if (!radiusExplorer_.IsExplored()) {
    ui_->Warn();
    return; /* 未探索 */
  }

  fputc(2, stdout);
  for (auto &l : veloMap) {
    fprintf(stdout, "%f, %f\n", l.distance, l.yaw);
  }
  fputc(3, stdout);
  fflush(stdout);

  ui_->SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
}

/* 計算した加減速をログとして出力 */
void Trace::PrintRadiusVelocityLog() {
  auto &veloMap = velocityMap_.Get();
  if (!velocityMap_.IsGenerated()) {
    ui_->Warn();
    return; /* 未計算 */
  }

  fputc(2, stdout);
  for (auto &v : veloMap) {
    fprintf(stdout, "%f\n", v);
  }
  fputc(3, stdout);
  fflush(stdout);
  ui_->SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
}

/* 補正位置を出力 */
void Trace::PrintPositionCorrectorLog() {
  fputc(2, stdout);
  {
    auto &cur = positionCorrector_.Get(Landmark::kCurvatureMark);
    fprintf(stdout, "Curvature:\n");
    if (cur.size() > 0) {
      for (auto &c : cur) {
        fprintf(stdout, "%f\n", c);
      }
    }
  }
  ui_->SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
  {
    auto &cro = positionCorrector_.Get(Landmark::kCrossLine);
    fprintf(stdout, "Cross:\n");
    if (cro.size() > 0) {
      for (auto &c : cro) {
        fprintf(stdout, "%f\n", c);
      }
    }
  }
  vTaskDelay(pdMS_TO_TICKS(1000));
  fputc(3, stdout);
  fflush(stdout);
  ui_->SetBuzzer(kBuzzerFrequency, kBuzzerEnterDuration);
}

#pragma GCC diagnostic pop
