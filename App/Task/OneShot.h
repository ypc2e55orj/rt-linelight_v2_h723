#ifndef TASK_ONESHOTTASK_H_
#define TASK_ONESHOTTASK_H_

/* Project */
#include "Wrapper/Task.h"

/**
 * ワンショットタスク基底クラス
 * (要求毎にレスポンスを返す)
 */
template <typename Class, typename Request, typename Response>
class OneShot : public Task<Class> {
 public:
  /* コンストラクタ */
  OneShot() {
    requestQueue_ = xQueueCreateStatic(1, sizeof(Request), requestQueueQueueStorageBuffer_, &requestQueueBuffer_);
    responseQueue_ = xQueueCreateStatic(1, sizeof(Response), responseQueueQueueStorageBuffer_, &responseQueueBuffer_);
  }

 protected:
  /* リクエスト送信 */
  bool Send(const Request &request, TickType_t xTicksToWait = portMAX_DELAY) {
    return xQueueSend(requestQueue_, &request, xTicksToWait) == pdTRUE;
  }

  /* 結果を取得 */
  bool Receive(Response &response, TickType_t xTicksToWait = portMAX_DELAY) {
    return xQueueReceive(responseQueue_, &response, xTicksToWait) == pdTRUE;
  }

  /* 要求時コールバック */
  virtual void OnRequest(const Request &request, Response &response) = 0;

  /* タスク */
  void TaskEntry() final {
    Request request = {};
    Response response = {};
    while (true) {
      if (xQueueReceive(requestQueue_, &request, portMAX_DELAY) == pdTRUE) {
        OnRequest(request, response);
        xQueueSend(responseQueue_, &response, portMAX_DELAY);
      }
    }
  }

 private:
  uint8_t requestQueueQueueStorageBuffer_[sizeof(Request)];
  StaticQueue_t requestQueueBuffer_;
  QueueHandle_t requestQueue_;
  uint8_t responseQueueQueueStorageBuffer_[sizeof(Response)];
  StaticQueue_t responseQueueBuffer_;
  QueueHandle_t responseQueue_;
};

#endif  // TASK_ONESHOTTASK_H_
