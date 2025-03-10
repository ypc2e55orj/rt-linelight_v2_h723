#ifndef APP_TRACE_H_
#define APP_TRACE_H_

/* FreeRTOS */
#include <FreeRTOS.h>

/* Project */
#include "Data/Pid.h"
#include "Data/Singleton.h"
#include "Fram.h"
#include "LineSensing/LineSensing.h"
#include "MotionPlaning/MotionPlaning.h"
#include "MotionPlaning/Suction.h"
#include "MotionPlaning/VelocityMapping.h"
#include "MotionSensing/MotionSensing.h"
#include "PowerMonitoring/PowerMonitoring.h"
#include "Ui.h"
#include "Wrapper/New.h"

/* C++ */
#include <vector>

class Trace : public Singleton<Trace> {
 public:
  /* 走行モード */
  enum Mode {
    kSearchRunning,
    kFastRunning,
  };
  /* 走行パラメータ */
  struct Parameter {
    Mode mode;               /* モード */
    uint32_t logInterval;    /* ログ出力周期 [ms] */
    float maxVelocity;       /* 探索上限速度・最短時初期速度 [m/s] */
    float acceleration;      /* 加速度 [m/ss] */
    float deceleration;      /* 最短時減速度 [m/ss] */
    Pid::Gain linearGain;    /* 並進PIDゲイン */
    Pid::Gain angularGain;   /* 旋回PIDゲイン */
    Pid::Gain lineErrorGain; /* ライン追従PIDゲイン */
    float stopDistance;      /* ゴールマーカーから停止までの距離 [m] */
    float suctionVoltage;    /* 吸引電圧 [V] */
  };

  /* コンストラクタ */
  Trace();

  /* 走行 */
  void Run(const Parameter &param);

  /* 不揮発メモリから探索データを読み込み */
  void LoadSearchRunningPoints();

  /* 速度マップを計算 */
  void CalculateVelocityMap(const std::vector<float> &minRadius, const std::vector<float> &maxVelocity,
                            float startVelocity, float acceleration, float deceleration);

  /* ログを出力 */
  void PrintLog();

  /* 探索した距離と角度をログとして出力 */
  void PrintSearchRunningPoints();

  /* 計算した加減速をログとして出力 */
  void PrintVelocityTable();

 private:
  /* 走行状態 */
  enum State {
    kStateResetting,
    kStateStartWaiting,
    kStateStarted,
    kStateGoalWaiting,
    kStateGoaled,
    kStateGoaledStopWaiting,
    kStateGoaledStopped,
    kStateEmergencyStop,
  };

  /* ログ */
#pragma pack(push, 1)
  struct Log {
    uint32_t time;
    LineSensing::LineImpl::State line;
    float commandVelocity;
    float estimateVelocity;
    float expectTranslate;
    float estimateTranslate;
    float correctedTranslate;
    float errorAngle;
    float commandAngularVelocity;
    float commandAngularVelocityP;
    float commandAngularVelocityI;
    float commandAngularVelocityD;
    float estimateAngularVelocity;
    float estimateRotate;
    float batteryVoltage;
    float motorVoltageRight;
    float motorVoltageLeft;
    float motorCurrentRight;
    float motorCurrentLeft;
    float x;
    float y;
    float theta;
    LineSensing::MarkerImpl::State markerRight;
    LineSensing::MarkerImpl::State markerLeft;
  };
#pragma pack(pop)

  /* 使用するクラス */
  MotionSensing::MotionSensing *ms_{nullptr};
  LineSensing::LineSensing *ls_{nullptr};
  MotionPlaning::MotionPlaning *mp_{nullptr};
  Ui *ui_{nullptr};
  Fram *fram_{nullptr};

  const PowerMonitoring::PowerImpl *power_{nullptr};
  MotionPlaning::Suction *suction_{nullptr};
  MotionPlaning::ServoImpl *servo_{nullptr};
  MotionSensing::OdometryImpl *odometry_{nullptr};
  const LineSensing::LineImpl *line_{nullptr};
  const LineSensing::MarkerImpl *marker_{nullptr};

  /* 走行パラメータ */
  Parameter param_{};
  /* 走行状態 */
  State state_{kStateResetting};
  /* リセットタイマー */
  uint32_t resetCount_{0};

  /* 加減速生成 */
  MotionPlaning::VelocityMapping velocityMap_;

  /* 速度・角速度 */
  float acceleration_{0.0f};    /* 加速度 [m/ss] */
  float maxVelocity_{0.0f};     /* 上限速度 [m/s] */
  float minVelocity_{0.0f};     /* 下限速度 [m/s] */
  float velocity_{0.0f};        /* 速度 [m/s] */
  float angularVelocity_{0.0f}; /* 角速度 [rad/s] */
  Pid lineErrorPid_{};          /* ライン追従PID */

  /* ログ */
  Log log_{};                     /* ログ一時バッファ */
  uint32_t logFrequencyCount_{0}; /* ログ出力周期カウンタ */
  uint32_t logBytes_{0};
  uint32_t logStartTime_{0};
  bool logEnabled_{false};

  /* 操作者に確認 */
  bool Confirmed();

  /* 緊急状態かどうか */
  bool CheckEmergency();
  /* 走行状態を更新 */
  void UpdateState();

  /* リセット */
  void OnResetting();
  /* スタートマーカーを待つ */
  void OnStartWaiting();
  /* スタートマーカー通過 */
  void OnStarted();
  /* ゴールマーカーを待つ */
  void OnGoalWaiting();
  /* ゴールマーカー通過 */
  void OnGoaled();
  /* 減速中 */
  void OnGoaledStopWaiting();
  /* 停止 */
  void OnGoaledStopped();

  /* 走行制御を更新 */
  void UpdateMotion();
  /* 現在の速度から指定距離で停止する加速度を計算 */
  static float CalculateDeceleration(float velocity, float distance);

  /* ログを更新 */
  void UpdateLog();
};

#endif  // APP_TRACE_H
