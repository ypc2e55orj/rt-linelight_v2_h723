#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

/* FreeRTOS */
#include <FreeRTOS.h>

/* C++ */
#include <cmath>

/* ハードウェアの設定 */
constexpr float kRegulatorVoltage = 3.298f; /* 3.3V レギュレータ電圧[V] */
constexpr float kMachineWeight = 110.0e-3f; /* 機体重量[kg] */
constexpr float kWheelDiameter = 23.0e-3f;  /* 車輪直径[m] TODO: 測る方法を考える */
/* MEMO: 長い直尺の横を走らせて走行距離と内部計算値の差で補正 */
constexpr float kWheelRadius = kWheelDiameter / 2.0f; /* 車輪半径[m] */
constexpr float kGearRatio = 42.0f / 11.0f;           /* ギア比[spur/pinion] */
constexpr float kTreadWidth = 101.0e-3f;              /* トレッド幅[m] */

/* バッテリー */
constexpr float kBatteryVoltageAdcGain = 4.0f;           /* バッテリー電圧AD変換ゲイン */
constexpr float kBatteryVoltageLimitMin = 10.50f;        /* バッテリー下限電圧[V] */
constexpr float kBatteryVoltageLimitMax = 12.60f;        /* バッテリー上限電圧[V] */
constexpr uint32_t kBatteryVoltageNumMovingAverage = 16; /* バッテリー電圧移動平均サンプル数 */
constexpr uint32_t kBatteryErrorTime = 5000;             /* 異常とするバッテリー電圧下限以下の連続時間 [ms] */
constexpr uint32_t kPowerAdcErrorTime = 5000;            /* 異常とするADC取得失敗連続回数 */

/* モーター */
constexpr float kTorqueConstant = 4.83e-3f;                            /* モータートルク定数[N*m/A] */
constexpr float kMotorBackEmf = 1.0f / 1980.0f;                        /* モーター起電力定数[V/rpm] */
constexpr float kMotorResistance = 1.94f;                              /* モーター抵抗[Ω] */
constexpr float kMotorLimitVoltage = 12.6f;                            /* モーター上限電圧[V] */
constexpr float kMotorCurrentMeasureDivResistor = 4.99e3f;             /* モーター電流計測分圧抵抗[Ω] */
constexpr float kMotorCurrentMeasureOffset = kRegulatorVoltage / 2.0f; /* モーター電流計測オフセット[V] */
constexpr float kSuctionFanLimitVoltage = 3.7f;                        /* 吸引ファン上限電圧[V] */

/* エンコーダー */
constexpr uint32_t kEncoderNumMovingAverage = 4; /* エンコーダー移動平均サンプル数 */

/* FF項 */
constexpr float kFeedForwardLinearGain = 0.0f;  /* 並進方向 TODO: */
constexpr float kFeedForwardAngularGain = 0.0f; /* 旋回方向 TODO: */

/* サーボ */
constexpr float kServoErrorLinearGain = 0.5f;    /* 目標速度を元にした下限速度のゲイン */
constexpr uint32_t kServoErrorLinearTime = 500;  /* 異常とする下限速度未満連続時間[ms] */
constexpr float kServoErrorAngularGain = 0.5f;   /* 目標角速度を元にした下限角速度のゲイン */
constexpr uint32_t kServoErrorAngularTime = 500; /* 異常とする下限角速度未満連続時間[ms] */

/* 周期通知 (Periodic) */
constexpr float kPeriodicNotifyInterval = 1.0e-3f; /* センサー更新間隔[s] */

/* ラインセンサー */
constexpr uint32_t kLineNumCalibrationSample = 5000; /* ラインセンサーキャリブレーション時間[ms] */
constexpr uint32_t kLineNumErrorMovingAverage = 4;   /* ラインセンサーエラー角度移動平均サンプル数 */
constexpr uint8_t kLineCrossDetectNum = 8;           /* ラインセンサー交差とする反応センサ個数 */
constexpr float kLineDistanceFromCenter = 81.04e-3f; /* ラインセンサーから車軸までの距離[m] */
constexpr float kLineDistanceFromMarker = 49.63e-3f; /* ラインセンサーからマーカーセンサーまでの距離[m] */
constexpr float kLineBrownOutIgnoreDistance = 0.1f;  /* ラインセンサーブラウンアウト無視距離[m] */
constexpr float kLineDetectThreshold = 0.6f;         /* ラインセンサー検知しきい値 */
constexpr float kMarkerDetectDistance = 0.010f;      /* マーカー検知距離[m] */
constexpr uint32_t kMarkerNumMovingAverage = 4;      /* ラインセンサー移動平均サンプル数 */
constexpr float kMarkerDetectThreshold = 0.5f;       /* マーカーセンサー検知しきい値 */
constexpr float kMarkerIgnoreOffset = 0.05f;         /* マーカー検知無視オフセット[m] */

/* ライン記憶 */
constexpr float kMappingLimitLength = 60.0f; /* 最大コース記憶距離[m] */
constexpr float kMappingDistance = 0.01f;    /* 曲率マップ解像度[m] */
constexpr float kMappingMaxRadius = 5.0f;    /* 最大曲率半径[m] */
constexpr float kMappingMinAngle = 0.00001f; /* 最小角度[rad] */

/* 位置補正 */
constexpr float kCorrectionAllowErrorCurvature = 0.1f; /* 曲率補正許容誤差 [m] */
constexpr float kCorrectionAllowErrorCrossLine = 0.1f; /* 交差補正許容誤差 [m] */

/* UI */
constexpr uint32_t kButtonShortPressThreshold = 100; /* 短押しきい値[ms] */
constexpr uint32_t kButtonLongPressThreshold = 1000; /* 長押しきい値[ms] */
constexpr uint16_t kBuzzerFrequency = 4000;          /* ブザー周波数[Hz] */
constexpr uint16_t kBuzzerEnterDuration = 100;       /* ブザーエンター鳴動時間[ms] */
constexpr uint16_t kBuzzerCancelDuration = 25;       /* ブザーキャンセル鳴動時間[ms] */
constexpr float kModeSelectWheelSpeed = 0.1f;        /* モード選択車輪速度[m/s] */

/* タスク優先度の設定 */
enum : UBaseType_t {
  kPriorityIdle = 0,
  kPriorityLow = 1,
  kPriorityBelowNormal = 2,
  kPriorityNormal = 3,
  kPriorityAboveNormal = 4,
  kPriorityHigh = 5,
  kPriorityRealtime = 6,
  kPriorityError = 7,
};
constexpr UBaseType_t kPriorityDefault = kPriorityNormal;           /* デフォルト(STM32CubeMXで設定) */
constexpr UBaseType_t kPriorityCom = kPriorityNormal;               /* 通信 */
constexpr UBaseType_t kPriorityFram = kPriorityNormal;              /* FRAM */
constexpr UBaseType_t kPriorityUi = kPriorityAboveNormal;           /* UI */
constexpr UBaseType_t kPriorityLineSensing = kPriorityHigh;         /* ライン計測 */
constexpr UBaseType_t kPriorityMotionSensing = kPriorityHigh;       /* 動作計測 */
constexpr UBaseType_t kPriorityMotionPlaning = kPriorityHigh;       /* 動作計画 */
constexpr UBaseType_t kPriorityPeriodic = kPriorityRealtime;        /* 1ms通知 */
constexpr UBaseType_t kPriorityPowerMonitoring = kPriorityRealtime; /* 電力監視 */

#endif  // APP_CONFIG_H_
