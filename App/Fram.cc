#include "Fram.h"

/* Project */
#include "Config.h"

/* C++ */
#include <algorithm>
#include <cstring>

/* グローバル変数定義 */
extern SPI_HandleTypeDef hspi4;

/* チップセレクト */
void Fram::SetChipSelect(bool status) {
  HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, status ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
/* ID読み出し */
uint32_t Fram::ReadId() {
  uint8_t cmd = kOpeCodeReadId;
  uint8_t id[4] = {0};
  HAL_StatusTypeDef ret = HAL_OK;

  SetChipSelect(true);
  ret = HAL_SPI_Transmit(&hspi4, &cmd, 1, HAL_MAX_DELAY);
  if (ret == HAL_OK) {
    ret = HAL_SPI_Receive(&hspi4, id, 4, HAL_MAX_DELAY);
  }
  SetChipSelect(false);

  if (ret == HAL_OK) {
    return (id[0] << 24) | (id[1] << 16) | (id[2] << 8) | id[3];
  }
  return 0;
}
/* ステータスレジスタ読み出し */
uint8_t Fram::ReadStatus() {
  uint8_t cmd = kOpeCodeReadStatusRegister;
  uint8_t status = 0;
  HAL_StatusTypeDef ret = HAL_OK;

  SetChipSelect(true);
  ret = HAL_SPI_Transmit(&hspi4, &cmd, 1, HAL_MAX_DELAY);
  if (ret == HAL_OK) {
    ret = HAL_SPI_Receive(&hspi4, &status, 1, HAL_MAX_DELAY);
  }
  SetChipSelect(false);

  if (ret == HAL_OK) {
    return status;
  }
  return 0x01;
}
/* 書き込み許可切り替え */
bool Fram::SetWriteLatch(bool enable) {
  uint8_t cmd = enable ? kOpeCodeWriteEnable : kOpeCodeWriteDisable;
  uint8_t expect = enable ? 0x02 : 0x00;
  HAL_StatusTypeDef ret = HAL_OK;

  SetChipSelect(true);
  ret = HAL_SPI_Transmit(&hspi4, &cmd, 1, HAL_MAX_DELAY);
  SetChipSelect(false);
  if (ret == HAL_OK) {
    uint8_t status = ReadStatus();
    return (status & 0x02) == expect;
  }
  return false;
}

/* DMA TXコールバック */
void Fram::TxCpltCallback(SPI_HandleTypeDef *) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(Fram::Instance().txCpltSemphr_, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/* DMA RXコールバック */
void Fram::RxCpltCallback(SPI_HandleTypeDef *) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(Fram::Instance().rxCpltSemphr_, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* コンストラクタ */
Fram::Fram()
    : txCpltSemphr_(xSemaphoreCreateBinaryStatic(&txCpltSemphrBuf_)),
      rxCpltSemphr_(xSemaphoreCreateBinaryStatic(&rxCpltSemphrBuf_)) {}

/* 初期化 */
bool Fram::Initialize() {
  if ((ReadId() & 0xffffff00) != 0x047f4900) {
    return false;
  }
  if (!SetWriteLatch(true)) {
    return false;
  }

  HAL_SPI_RegisterCallback(&hspi4, HAL_SPI_TX_COMPLETE_CB_ID, TxCpltCallback);
  HAL_SPI_RegisterCallback(&hspi4, HAL_SPI_RX_COMPLETE_CB_ID, RxCpltCallback);

  return TaskCreate("Fram", configMINIMAL_STACK_SIZE, kPriorityFram);
}

/* 書き込み */
bool Fram::Write(uint32_t address, const void *data, uint32_t size, TickType_t xTicksToWait) {
  FramRequest request = {FramRequest::Type::kWrite, address, reinterpret_cast<const uint8_t *>(data), nullptr, size};
  FramResponse response = {};

  if (address + size > kMaxAddress) {
    return false;
  }
  if (Send(request, xTicksToWait) && Receive(response, xTicksToWait)) {
    return response == FramResponse::kSuccess;
  }
  return false;
}
/* 読み出し */
bool Fram::Read(uint32_t address, void *data, uint32_t size, TickType_t xTicksToWait) {
  FramRequest request = {FramRequest::Type::kRead, address, nullptr, reinterpret_cast<uint8_t *>(data), size};
  FramResponse response = {};

  if (address + size > kMaxAddress) {
    return false;
  }
  if (Send(request, xTicksToWait) && Receive(response, xTicksToWait)) {
    return response == FramResponse::kSuccess;
  }
  return false;
}
/* クリア */
bool Fram::Clear() {
  FramRequest request = {FramRequest::Type::kClear, 0, nullptr, nullptr, 0};
  FramResponse response = {};

  if (Send(request) && Receive(response)) {
    return response == FramResponse::kSuccess;
  }
  return false;
}

/* 書き込み要求時 */
FramResponse Fram::OnWriteRequest(const FramRequest &request) {
  FramResponse response = FramResponse::kSuccess;
  memset(buffer_, 0, sizeof(buffer_));
  for (uint32_t txTotal = 0; txTotal < request.size;) {
    uint32_t address = request.address + txTotal;
    uint32_t tx = std::min(kBufferSize, request.size - txTotal);
    buffer_[0] = kOpeCodeWrite;
    buffer_[1] = (address >> 16) & 0xff;
    buffer_[2] = (address >> 8) & 0xff;
    buffer_[3] = address & 0xff;
    memcpy(buffer_ + kCommandSize, request.writePtr + txTotal, tx);
    SCB_CleanDCache_by_Addr(reinterpret_cast<uint32_t *>(buffer_), kAlignedBufferSize);
    SetChipSelect(true);
    if (HAL_SPI_Transmit_DMA(&hspi4, buffer_, static_cast<uint16_t>(kCommandSize + tx)) == HAL_OK) {
      xSemaphoreTake(txCpltSemphr_, portMAX_DELAY);
    } else {
      response = FramResponse::kFailure;
    }
    SetChipSelect(false);
    txTotal += tx;
  }
  return response;
}
/* 読み出し要求時 */
FramResponse Fram::OnReadRequest(const FramRequest &request) {
  FramResponse response = FramResponse::kSuccess;
  memset(buffer_, 0, sizeof(buffer_));
  for (uint32_t rxTotal = 0; rxTotal < request.size;) {
    uint32_t address = request.address + rxTotal;
    uint32_t rx = std::min(kBufferSize, request.size - rxTotal);
    buffer_[0] = kOpeCodeRead;
    buffer_[1] = (address >> 16) & 0xff;
    buffer_[2] = (address >> 8) & 0xff;
    buffer_[3] = address & 0xff;
    SCB_CleanDCache_by_Addr(reinterpret_cast<uint32_t *>(buffer_), kAlignedBufferSize);
    SetChipSelect(true);
    if (HAL_SPI_Transmit_DMA(&hspi4, buffer_, kCommandSize) == HAL_OK) {
      xSemaphoreTake(txCpltSemphr_, portMAX_DELAY);
    } else {
      response = FramResponse::kFailure;
    }
    if (HAL_SPI_Receive_DMA(&hspi4, buffer_, static_cast<uint16_t>(rx)) == HAL_OK) {
      xSemaphoreTake(rxCpltSemphr_, portMAX_DELAY);
    } else {
      response = FramResponse::kFailure;
    }
    SetChipSelect(false);
    SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t *>(buffer_), kAlignedBufferSize);
    memcpy(request.readPtr + rxTotal, buffer_, rx);
    rxTotal += rx;
  }
  return response;
}
/* 全消去要求時 */
FramResponse Fram::OnClearRequest() {
  FramResponse response = FramResponse::kSuccess;
  memset(buffer_, 0, sizeof(buffer_));
  for (uint32_t address = 0; address < kMaxAddress;) {
    buffer_[0] = kOpeCodeWrite;
    buffer_[1] = (address >> 16) & 0xff;
    buffer_[2] = (address >> 8) & 0xff;
    buffer_[3] = address & 0xff;
    SCB_CleanDCache_by_Addr(reinterpret_cast<uint32_t *>(buffer_), kAlignedBufferSize);
    SetChipSelect(true);
    if (HAL_SPI_Transmit_DMA(&hspi4, buffer_, kCommandSize + kBufferSize) == HAL_OK) {
      xSemaphoreTake(txCpltSemphr_, portMAX_DELAY);
    } else {
      response = FramResponse::kFailure;
    }
    SetChipSelect(false);
    address += kBufferSize;
  }
  return response;
}

/* 要求時 */
void Fram::OnRequest(const FramRequest &request, FramResponse &response) {
  switch (request.type) {
    case FramRequest::Type::kWrite:
      response = OnWriteRequest(request);
      break;
    case FramRequest::Type::kRead:
      response = OnReadRequest(request);
      break;
    case FramRequest::Type::kClear:
      response = OnClearRequest();
      break;
  }
}
