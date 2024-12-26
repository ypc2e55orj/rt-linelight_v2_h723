#ifndef APP_COM_H_
#define APP_COM_H_

/* STM32CubeMX */
#include <main.h>

/* C++ */
#include <cstdio>

/* Projcet */
#include "Task/OneShot.h"

struct ComRequest {
  enum class Type {
    kWrite, /* 書き込み */
    kRead,  /* 読み込み */
  };
  Type type;               /* 要求タイプ */
  const uint8_t *writePtr; /* 書き込み元バッファ */
  uint8_t *readPtr;        /* 読み込み先バッファ */
  uint32_t size;           /* サイズ */
};
enum class ComResponse {
  kSuccess,
  kFailure,
};

class Com final : public OneShot<Com, ComRequest, ComResponse> {
 private:
  /* DMA TXコールバック */
  static void TxCpltCallback(UART_HandleTypeDef *);
  /* DMA RXコールバック */
  static void RxCpltCallback(UART_HandleTypeDef *);

  /* DMAバッファ */
  static constexpr uint32_t kBufferSize = 1024;  /* 最大処理サイズ */
  static constexpr uint32_t kAlignedBufferSize = /* 最大処理サイズ (32バイトアライン) */
      ((kBufferSize + 31) / 32) * 32;
  ALIGN_32BYTES(uint8_t buffer_[kAlignedBufferSize]);

  StaticSemaphore_t txCpltSemphrBuf_; /* 送信完了セマフォバッファ */
  SemaphoreHandle_t txCpltSemphr_;    /* 送信完了セマフォ */
  StaticSemaphore_t rxCpltSemphrBuf_; /* 受信完了セマフォバッファ */
  SemaphoreHandle_t rxCpltSemphr_;    /* 受信完了セマフォ */

 public:
  /* コンストラクタ */
  Com();

  /* シリアル通信を初期化 */
  bool Initialize();

  /* 書き込み */
  bool Write(const void *data, uint32_t size);

  /* 読み出し */
  bool Read(void *data, uint32_t size);

 private:
  /* 要求時 */
  void OnRequest(const ComRequest &request, ComResponse &response) final;
  /* 書き込み要求時 */
  ComResponse OnWriteRequest(const ComRequest &request);
  /* 読み出し要求時 */
  ComResponse OnReadRequest(const ComRequest &request);
};

#ifdef __cplusplus
extern "C" {
#endif

int _write(int, char *ptr, int len);
int _read(int, char *ptr, int len);

#ifdef __cplusplus
}
#endif

#endif  // APP_COM_H_
