#include "MotionSensing/Imu.h"

/* STM32CubeMX */
#include <main.h>

/* FreeRTOS */
#include <FreeRTOS.h>
#include <semphr.h>

/* C++ */
#include <cstring>
#include <mutex>
#include <numbers>

/* Project */
#include "MotionSensing/lsm6dsrx_reg.h"

/* グローバル変数定義 */
extern SPI_HandleTypeDef hspi2;

namespace MotionSensing {
/**
 * MARK: Lsm6dsrx
 */

class Lsm6dsrx : public Singleton<Lsm6dsrx> {
 public:
  enum Value : uint32_t {
    kTemp,
    kGyroX,
    kGyroY,
    kGyroZ,
    kAccelX,
    kAccelY,
    kAccelZ,
    kNumValue,
  };

  /* コンストラクタ */
  Lsm6dsrx();

  /* 初期化 */
  bool Initialize();

  /* 値を更新 */
  bool Fetch();

  /* データ取得(生値) */
  int16_t GetRaw(uint32_t value);

  /* 単位系に変換 */
  static float ConvertDegC(int32_t tempRaw) { return static_cast<float>(tempRaw) / 256.0f; }
  static float ConvertRadPerSec(int32_t gyroRaw) {
    return static_cast<float>(gyroRaw) * kSensitivityGyro * std::numbers::pi_v<float> / 180.0f;
  }
  static float ConvertGravity(int32_t accelRaw) { return static_cast<float>(accelRaw) * kSensitivityAccel; }
  static float ConvertMeterPerSec2(int32_t accelRaw) { return ConvertGravity(accelRaw) * 9.80665f; }

 private:
  static constexpr float kSensitivityGyro = 140.0f / 1000.0f;  /* 4000 [deg/s]; 0.140   [deg/s/LSB] */
  static constexpr float kSensitivityAccel = 0.244f / 1000.0f; /* 8        [G]; 0.00244 [G/LSB] */

  StaticSemaphore_t txCpltSemphrBuffer_; /* 送信完了セマフォバッファ */
  SemaphoreHandle_t txCpltSemphr_;       /* 送信完了セマフォ */
  StaticSemaphore_t rxCpltSemphrBuffer_; /* 受信完了セマフォバッファ */
  SemaphoreHandle_t rxCpltSemphr_;       /* 受信完了セマフォ */
  ALIGN_32BYTES(uint8_t txBuffer_[32]);  /* 送信バッファ */
  ALIGN_32BYTES(uint8_t rxBuffer_[32]);  /* 受信バッファ */

  /* 送信完了コールバック */
  static void TxCpltCallback(SPI_HandleTypeDef *);
  /* 受信完了コールバック */
  static void RxCpltCallback(SPI_HandleTypeDef *);

  /* チップセレクト */
  static void ChipSelect(bool enable);

  /* レジスタに設定を書き込み */
  static void WriteReg(uint8_t addr, uint8_t data);

  /* レジスタから設定を読み込み */
  static uint8_t ReadReg(uint8_t addr);

  /* LSM6DSRXを設定 */
  bool ConfigureLsm6dsrx();
};

/* チップセレクト */
void Lsm6dsrx::ChipSelect(bool enable) {
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
/* レジスタに設定を書き込み */
void Lsm6dsrx::WriteReg(uint8_t addr, uint8_t data) {
  ChipSelect(true);
  HAL_SPI_Transmit(&hspi2, &addr, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&hspi2, &data, 1, HAL_MAX_DELAY);
  ChipSelect(false);
}
/* レジスタから設定を読み込み */
uint8_t Lsm6dsrx::ReadReg(uint8_t addr) {
  uint8_t data = 0;
  addr |= 0x80;
  ChipSelect(true);
  HAL_SPI_Transmit(&hspi2, &addr, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi2, &data, 1, HAL_MAX_DELAY);
  ChipSelect(false);
  return data;
}

/* 送信完了コールバック */
void Lsm6dsrx::TxCpltCallback(SPI_HandleTypeDef *) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(Lsm6dsrx::Instance().txCpltSemphr_, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* 受信完了コールバック */
void Lsm6dsrx::RxCpltCallback(SPI_HandleTypeDef *) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(Lsm6dsrx::Instance().rxCpltSemphr_, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* LSM6DSRXを設定 */
bool Lsm6dsrx::ConfigureLsm6dsrx() {
  lsm6dsrx_reg_t reg = {};

  /* WHO_AM_I */
  while (ReadReg(LSM6DSRX_WHO_AM_I) != LSM6DSRX_ID) {
    vTaskDelay(1);
  }
  /* リセット */
  reg.byte = 0;
  reg.ctrl3_c.sw_reset = 1;
  WriteReg(LSM6DSRX_CTRL3_C, reg.byte);
  while (ReadReg(LSM6DSRX_CTRL3_C) & 0x01) {
    vTaskDelay(1);
  }
  /* CTRL3_C 設定 */
  reg.byte = 0;
  reg.ctrl3_c.if_inc = 1; /* Register address automatically incremented */
  /* during a multiple byte access with a serial interface (I²C or SPI). */
  reg.ctrl3_c.bdu = 1; /* Block Data Update. */
  WriteReg(LSM6DSRX_CTRL3_C, reg.byte);
  /* CTRL1_XL 設定 (加速度計) */
  reg.byte = 0;
  reg.ctrl1_xl.odr_xl = LSM6DSRX_XL_ODR_1666Hz; /* Accelerometer ODR selection */
  reg.ctrl1_xl.fs_xl = LSM6DSRX_8g;             /* Accelerometer full-scale selection. */
  /* (00: ±2 g; 01: ±16 g; 10: ±4 g; 11: ±8 g) */
  reg.ctrl1_xl.lpf2_xl_en = 1; /* Accelerometer high-resolution selection */
  /* 0: output from first stage digital filtering selected */
  /* 1: output from LPF2 second filtering stage selected */
  WriteReg(LSM6DSRX_CTRL1_XL, reg.byte);
  /* LPF2 設定 */
  reg.byte = 0;
  reg.ctrl8_xl.hpcf_xl = LSM6DSRX_LP_ODR_DIV_45; /* Accelerometer LPF2 and HP filter configuration
                                                    and cutoff setting. */
                                                 /* Refer to Table 68. */
  WriteReg(LSM6DSRX_CTRL8_XL, reg.byte);
  /* CTRL2_G 設定 (ジャイロ) */
  reg.byte = 0;
  reg.ctrl2_g.odr_g = LSM6DSRX_GY_ODR_1666Hz;
  reg.ctrl2_g.fs_g = LSM6DSRX_4000dps;
  WriteReg(LSM6DSRX_CTRL2_G, reg.byte);

  /* データ取得用の送信データを準備 */
  memset(txBuffer_, 0, sizeof(txBuffer_));
  txBuffer_[0] = 0x80 | LSM6DSRX_OUT_TEMP_L;
  SCB_CleanDCache_by_Addr(reinterpret_cast<uint32_t *>(txBuffer_), sizeof(txBuffer_));

  return true;
}

/* コンストラクタ */
Lsm6dsrx::Lsm6dsrx()
    : txCpltSemphr_(xSemaphoreCreateBinaryStatic(&txCpltSemphrBuffer_)),
      rxCpltSemphr_(xSemaphoreCreateBinaryStatic(&rxCpltSemphrBuffer_)) {}

/* 初期設定 */
bool Lsm6dsrx::Initialize() {
  bool lsm = ConfigureLsm6dsrx();
  bool txCb = HAL_SPI_RegisterCallback(&hspi2, HAL_SPI_TX_COMPLETE_CB_ID, TxCpltCallback) == HAL_OK;
  bool rxCb = HAL_SPI_RegisterCallback(&hspi2, HAL_SPI_RX_COMPLETE_CB_ID, RxCpltCallback) == HAL_OK;
  return lsm && txCb && rxCb;
}

/* データ取得 */
bool Lsm6dsrx::Fetch() {
  bool ret = false;
  ChipSelect(true);
  /* 読み出すアドレスを送信 (アドレスは自動でインクリメントされる) */
  if (HAL_SPI_Transmit_DMA(&hspi2, txBuffer_, 1) == HAL_OK) {
    if (xSemaphoreTake(txCpltSemphr_, pdMS_TO_TICKS(1)) == pdTRUE) {
      /* 読み出し */
      if (HAL_SPI_Receive_DMA(&hspi2, rxBuffer_, kNumValue * 2) == HAL_OK) {
        if (xSemaphoreTake(rxCpltSemphr_, pdMS_TO_TICKS(1)) == pdTRUE) {
          /* キャッシュを更新 */
          SCB_CleanInvalidateDCache_by_Addr(reinterpret_cast<uint32_t *>(rxBuffer_), sizeof(rxBuffer_));
          ret = true;
        }
      }
    }
  }
  ChipSelect(false);
  return ret;
}

/* データ取得(生値) */
int16_t Lsm6dsrx::GetRaw(uint32_t value) { return reinterpret_cast<int16_t *>(rxBuffer_)[value]; }

/**
 * MARK: Imu
 */
/* 初期化 */
bool Imu::Initialize() {
  Reset();
  return Lsm6dsrx::Instance().Initialize();
}

/* 内部値を更新(1ms周期で呼び出すこと) */
bool Imu::Update() {
  auto &lsm = Lsm6dsrx::Instance();
  if (!lsm.Fetch()) {
    return false;
  }
  {
    std::scoped_lock<Mutex> lock{mtx_};
    gyro_[0] = lsm.GetRaw(Lsm6dsrx::kGyroX) - offset_[0];
    gyro_[1] = lsm.GetRaw(Lsm6dsrx::kGyroY) - offset_[1];
    gyro_[2] = lsm.GetRaw(Lsm6dsrx::kGyroZ) - offset_[2];
    accel_[0] = lsm.GetRaw(Lsm6dsrx::kAccelX) - offset_[3];
    accel_[1] = lsm.GetRaw(Lsm6dsrx::kAccelY) - offset_[4];
    accel_[2] = lsm.GetRaw(Lsm6dsrx::kAccelZ) - offset_[5];
  }
  return true;
}

/* リセット */
void Imu::Reset() {
  std::scoped_lock<Mutex> lock{mtx_};
  gyro_.fill(0);
  accel_.fill(0);
}

/* オフセットを取得 */
void Imu::GetOffset(Offset &offset) {
  std::scoped_lock<Mutex> lock{mtx_};
  offset = offset_;
}

/* オフセットを設定 */
void Imu::SetOffset(const Offset &offset) {
  std::scoped_lock<Mutex> lock{mtx_};
  offset_ = offset;
}

/* x軸加速度を取得 [m/ss] */
float Imu::GetAccelX() { return Lsm6dsrx::Instance().ConvertMeterPerSec2(accel_[0]); }

/* y軸加速度を取得 [m/ss] */
float Imu::GetAccelY() { return Lsm6dsrx::Instance().ConvertMeterPerSec2(accel_[1]); }

/* ヨーレートを取得 */
float Imu::GetYawRate() { return Lsm6dsrx::Instance().ConvertRadPerSec(gyro_[2]); }

/* 加速度センサの生値を取得 */
Imu::Raw Imu::GetAccelRaw() {
  std::scoped_lock<Mutex> lock{mtx_};
  return accel_;
}

/* ジャイロセンサの生値を取得 */
Imu::Raw Imu::GetGyroRaw() {
  std::scoped_lock<Mutex> lock{mtx_};
  return gyro_;
}

}  // namespace MotionSensing
