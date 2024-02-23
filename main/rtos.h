#pragma once

// ESP-IDF
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

namespace rtos {
// キューラッパー
template <typename T>
class Queue {
 public:
  explicit Queue(UBaseType_t uxQueueLength) {
    queue_ = xQueueCreate(uxQueueLength, sizeof(T));
    assert(queue_ != nullptr);
  }
  ~Queue() { vQueueDelete(queue_); }

  void reset() { xQueueReset(queue_); }

  // 送信
  bool send(const T* item, TickType_t xTicksToWait) { return xQueueSend(queue_, item, xTicksToWait) == pdTRUE; }
  bool send_isr(const T* item, BaseType_t* pxHigherPriorityTaskWoken) {
    return xQueueSendFromISR(queue_, item, pxHigherPriorityTaskWoken) == pdTRUE;
  }
  bool send_front(const T* item, TickType_t xTickToWait) {
    return xQueueSendToFront(queue_, item, xTickToWait) == pdTRUE;
  }
  bool send_front_isr(const T* item, BaseType_t* pxHigherPriorityTaskWoken) {
    return xQueueSendToFrontFromISR(queue_, item, pxHigherPriorityTaskWoken) == pdTRUE;
  }

  // 上書き送信 (長さ1のキューで使う想定のメソッド)
  bool overwrite(const T* item) { return xQueueOverwrite(queue_, item) == pdTRUE; }
  bool overwrite_isr(const T* item, BaseType_t* pxHigherPriorityTaskWoken) {
    return xQueueOverwriteFromISR(queue_, item, pxHigherPriorityTaskWoken) == pdTRUE;
  }

  // 受信
  bool receive(T* const item, TickType_t xTicksToWait) { return xQueueReceive(queue_, item, xTicksToWait) == pdTRUE; }
  bool receive_isr(T* const item, BaseType_t* pxHigherPriorityTaskWoken) {
    return xQueueReceiveFromISR(queue_, item, pxHigherPriorityTaskWoken) == pdTRUE;
  }

  // 先頭アイテムを取得
  bool peek(T* item, TickType_t xTicksToWait) { return xQueuePeek(queue_, item, xTicksToWait) == pdTRUE; }
  bool peek_isr(T* item) { return xQueuePeekFromISR(queue_, item) == pdTRUE; }

  // 待ちアイテム数を取得
  UBaseType_t waiting() { return uxQueueMessagesWaiting(queue_); }
  UBaseType_t waiting_isr() { return uxQueueMessagesWaitingFromISR(queue_); }

  // 空きアイテム数を取得
  UBaseType_t available() { return uxQueueSpacesAvailable(queue_); }

 private:
  QueueHandle_t queue_;
};
}  // namespace rtos