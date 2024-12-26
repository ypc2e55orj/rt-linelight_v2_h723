#include "Com.h"

/* C++ */
#include <algorithm>
#include <cstring>

/* Project */
#include "Config.h"

/* グローバル変数定義 */
extern UART_HandleTypeDef huart4;

/* DMA TXコールバック */
void Com::TxCpltCallback(UART_HandleTypeDef *) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(Com::Instance().txCpltSemphr_, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/* DMA RXコールバック */
void Com::RxCpltCallback(UART_HandleTypeDef *) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(Com::Instance().rxCpltSemphr_, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* コンストラクタ */
Com::Com()
    : txCpltSemphr_(xSemaphoreCreateBinaryStatic(&txCpltSemphrBuf_)),
      rxCpltSemphr_(xSemaphoreCreateBinaryStatic(&rxCpltSemphrBuf_)) {}

/* シリアル通信を初期化 */
bool Com::Initialize() {
  HAL_UART_RegisterCallback(&huart4, HAL_UART_TX_COMPLETE_CB_ID, TxCpltCallback);
  HAL_UART_RegisterCallback(&huart4, HAL_UART_RX_COMPLETE_CB_ID, RxCpltCallback);

  return TaskCreate("Com", configMINIMAL_STACK_SIZE, kPriorityCom);
}

/* 書き込み */
bool Com::Write(const void *data, uint32_t size) {
  ComRequest request = {ComRequest::Type::kWrite, reinterpret_cast<const uint8_t *>(data), nullptr, size};
  ComResponse response = {};
  if (Send(request) && Receive(response)) {
    return response == ComResponse::kSuccess;
  }
  return false;
}

/* 読み出し */
bool Com::Read(void *data, uint32_t size) {
  ComRequest request = {ComRequest::Type::kRead, nullptr, reinterpret_cast<uint8_t *>(data), size};
  ComResponse response = {};
  if (Send(request) && Receive(response)) {
    return response == ComResponse::kSuccess;
  }
  return false;
}

/* 書き込み要求時 */
ComResponse Com::OnWriteRequest(const ComRequest &request) {
  ComResponse response = ComResponse::kSuccess;
  for (uint32_t txTotal = 0; txTotal < request.size;) {
    uint32_t tx = std::min(kBufferSize, request.size - txTotal);
    memcpy(buffer_, request.writePtr + txTotal, tx);
    SCB_CleanDCache_by_Addr(reinterpret_cast<uint32_t *>(buffer_), kAlignedBufferSize);
    if (HAL_UART_Transmit_DMA(&huart4, buffer_, static_cast<uint16_t>(tx)) == HAL_OK) {
      for (uint32_t txItm = 0; txItm < tx; ++txItm) {
        ITM_SendChar(buffer_[txItm]);
      }
      xSemaphoreTake(txCpltSemphr_, portMAX_DELAY);
    } else {
      response = ComResponse::kFailure;
    }
    txTotal += tx;
  }
  return response;
}

/* 読み出し要求時 */
ComResponse Com::OnReadRequest(const ComRequest &request) {
  ComResponse response = ComResponse::kSuccess;
  for (uint32_t rxTotal = 0; rxTotal < request.size;) {
    uint32_t rx = std::min(kBufferSize, request.size - rxTotal);
    if (HAL_UART_Receive_DMA(&huart4, buffer_, static_cast<uint16_t>(rx)) == HAL_OK) {
      xSemaphoreTake(rxCpltSemphr_, portMAX_DELAY);
    } else {
      response = ComResponse::kFailure;
    }
    SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t *>(buffer_), kAlignedBufferSize);
    memcpy(request.readPtr + rxTotal, buffer_, rx);
    rxTotal += rx;
  }
  return response;
}

/* 要求時 */
void Com::OnRequest(const ComRequest &request, ComResponse &response) {
  switch (request.type) {
    case ComRequest::Type::kWrite:
      response = OnWriteRequest(request);
      break;
    case ComRequest::Type::kRead:
      response = OnReadRequest(request);
      break;
  }
}

/* システムコールをオーバーロード */
extern "C" int _write(int, char *ptr, int len) {
  if (Com::Instance().Write(reinterpret_cast<const uint8_t *>(ptr), static_cast<uint32_t>(len))) {
    return len;
  }
  return -1;
}
extern "C" int _read(int, char *ptr, int len) {
  if (Com::Instance().Read(reinterpret_cast<uint8_t *>(ptr), static_cast<uint32_t>(len))) {
    return len;
  }
  return -1;
}
