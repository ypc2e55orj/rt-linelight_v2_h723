#ifndef APP_FRAM_H_
#define APP_FRAM_H_

/* STM32CubeMX */
#include <main.h>

/* Project */
#include "Task/OneShot.h"

struct FramRequest {
  enum class Type {
    kWrite, /* 書き込み */
    kRead,  /* 読み込み */
    kClear, /* 全消去 */
  };
  Type type;
  uint32_t address;        /* 開始アドレス */
  const uint8_t *writePtr; /* 書き込み元バッファ */
  uint8_t *readPtr;        /* 読み込み先バッファ */
  uint32_t size;           /* サイズ */
};
enum class FramResponse {
  kSuccess,
  kFailure,
};

class Fram final : public OneShot<Fram, FramRequest, FramResponse> {
 private:
  static constexpr uint32_t kCommandSize = 4;
  static constexpr uint32_t kBufferSize = 1024;  /* 最大処理サイズ */
  static constexpr uint32_t kAlignedBufferSize = /* 最大処理サイズ (32バイトアライン) */
      (((kCommandSize + kBufferSize) + 31) / 32) * 32;
  ALIGN_32BYTES(uint8_t buffer_[kAlignedBufferSize]);

  StaticSemaphore_t txCpltSemphrBuf_; /* 送信完了セマフォバッファ */
  SemaphoreHandle_t txCpltSemphr_;    /* 送信完了セマフォ */
  StaticSemaphore_t rxCpltSemphrBuf_; /* 受信完了セマフォバッファ */
  SemaphoreHandle_t rxCpltSemphr_;    /* 受信完了セマフォ */

  /* 送信完了コールバック */
  static void TxCpltCallback(SPI_HandleTypeDef *);
  /* 受信完了コールバック */
  static void RxCpltCallback(SPI_HandleTypeDef *);

 public:
  static constexpr uint32_t kMaxAddress = 0x7ffff; /* 最大アドレス */

  /* コンストラクタ */
  Fram();

  /* 初期化 */
  bool Initialize();

  /* 書き込み */
  bool Write(uint32_t address, const void *data, uint32_t size, TickType_t xTicksToWait = portMAX_DELAY);

  /* 読み出し */
  bool Read(uint32_t address, void *data, uint32_t size, TickType_t xTicksToWait = portMAX_DELAY);

  /* クリア */
  bool Clear();

 private:
  /* 要求時 */
  void OnRequest(const FramRequest &request, FramResponse &response) final;
  /* 書き込み要求時 */
  FramResponse OnWriteRequest(const FramRequest &request);
  /* 読み出し要求時 */
  FramResponse OnReadRequest(const FramRequest &request);
  /* 全消去要求時 */
  FramResponse OnClearRequest();

  /* オペコード */
  static constexpr uint8_t kOpeCodeWriteEnable = 0x06;
  static constexpr uint8_t kOpeCodeWriteDisable = 0x04;
  static constexpr uint8_t kOpeCodeReadStatusRegister = 0x05;
  static constexpr uint8_t kOpeCodeRead = 0x03;
  static constexpr uint8_t kOpeCodeWrite = 0x02;
  static constexpr uint8_t kOpeCodeReadId = 0x9f;

  /* チップセレクト */
  static void SetChipSelect(bool status);
  /* ID読み出し */
  static uint32_t ReadId();
  /* ステータスレジスタ読み出し */
  static uint8_t ReadStatus();
  /* 書き込み許可切り替え */
  static bool SetWriteLatch(bool enable);
};
#endif  // APP_FRAM_H_
