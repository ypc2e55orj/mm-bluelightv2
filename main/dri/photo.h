#pragma once

// C++
#include <array>
#include <memory>

// ESP-IDF
#include <driver/gptimer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Project
#include "adc.h"
#include "gpio.h"

class Photo {
 public:
  static constexpr size_t PHOTO_COUNTS = 4;

  struct Result {
    int ambient;
    int flash;
  };

  struct Config {
    adc_unit_t adc_unit;
    adc_channel_t adc_channel[4];
    gpio_num_t gpio_num[4];
  };

 private:
  static constexpr auto TAG = "driver::hardware::Photo::PhotoImpl";

  static constexpr uint32_t TIMER_RESOLUTION_HZ = 1'000'000;   // [Hz]
  static constexpr uint32_t INTERVAL_TIMER_FREQUENCY = 5'000;  // [Hz]
  static constexpr uint32_t INTERVAL_TIMER_COUNTS =
      TIMER_RESOLUTION_HZ / INTERVAL_TIMER_FREQUENCY;
  static constexpr uint32_t FLASH_TIMER_FREQUENCY = 10'000;  // [Hz]
  static constexpr uint32_t FLASH_TIMER_COUNTS =
      TIMER_RESOLUTION_HZ / FLASH_TIMER_FREQUENCY;

  // テーブルの添字
  static constexpr size_t LEFT90_POS = 0;
  static constexpr size_t LEFT45_POS = 1;
  static constexpr size_t RIGHT45_POS = 2;
  static constexpr size_t RIGHT90_POS = 3;

  // 取得中のセンサ位置
  uint8_t index_;
  // 発光タイマー
  gptimer_handle_t flash_timer_;
  // 受光タイマー
  gptimer_handle_t receive_timer_;

  // GPIOテーブル
  std::array<std::unique_ptr<Gpio>, PHOTO_COUNTS> gpio_;
  // ADCテーブル
  std::array<std::unique_ptr<Adc>, PHOTO_COUNTS> adc_;
  // 結果テーブル
  std::array<Result, PHOTO_COUNTS> result_;

  // 完了通知先
  TaskHandle_t task_;

  static bool IRAM_ATTR flash_callback(gptimer_handle_t,
                                       const gptimer_alarm_event_data_t *,
                                       void *user_ctx) {
    auto this_ptr = reinterpret_cast<Photo *>(user_ctx);
    // 読み取り
    this_ptr->adc_[this_ptr->index_]->read_isr(
        this_ptr->result_[this_ptr->index_].ambient);
    // 点灯
    this_ptr->gpio_[this_ptr->index_]->set(true);
    // 受光タイマー開始
    // ESP_ERROR_CHECK(gptimer_start(this_ptr->receive_timer_));
    gptimer_start(this_ptr->receive_timer_);
    // portYIELD_FROM_ISRと同等
    return false;
  }

  static bool IRAM_ATTR receive_callback(gptimer_handle_t timer,
                                         const gptimer_alarm_event_data_t *,
                                         void *user_ctx) {
    auto this_ptr = reinterpret_cast<Photo *>(user_ctx);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // 読み取り
    this_ptr->adc_[this_ptr->index_]->read_isr(
        this_ptr->result_[this_ptr->index_].flash);
    // 消灯
    this_ptr->gpio_[this_ptr->index_]->set(false);
    // タイマー停止
    // ESP_ERROR_CHECK(gptimer_stop(timer));
    gptimer_stop(timer);
    if (this_ptr->index_ == 0x03) {
      // ESP_ERROR_CHECK(gptimer_stop(this_ptr->flash_timer_));
      gptimer_stop(this_ptr->flash_timer_);
      vTaskNotifyGiveFromISR(this_ptr->task_,
                             &xHigherPriorityTaskWoken);  // NOLINT
    }
    // 更新
    this_ptr->index_ = (this_ptr->index_ + 1) & 0x03;
    return xHigherPriorityTaskWoken == pdTRUE;
  }

 public:
  explicit Photo(Config &config)
      : index_(0), flash_timer_(), receive_timer_(), result_(), task_() {
    // GPIO/ADCを初期化
    for (size_t i = 0; i < PHOTO_COUNTS; i++) {
      gpio_[i] = std::make_unique<Gpio>(config.gpio_num[i], GPIO_MODE_OUTPUT,
                                        false, true);
      adc_[i] = std::make_unique<Adc>(config.adc_unit, config.adc_channel[i]);
    }

    // 受光タイマー
    gptimer_config_t receive_config = {};
    receive_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    receive_config.direction = GPTIMER_COUNT_UP;
    receive_config.resolution_hz = TIMER_RESOLUTION_HZ;
    ESP_ERROR_CHECK(gptimer_new_timer(&receive_config, &receive_timer_));

    // 受光タイマー コールバックを登録
    gptimer_event_callbacks_t receive_callback_config = {};
    receive_callback_config.on_alarm = receive_callback;
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(
        receive_timer_, &receive_callback_config, this));

    // 受光タイマー コールバックが発火する条件を設定
    gptimer_alarm_config_t receive_alarm = {};
    receive_alarm.reload_count = 0;
    receive_alarm.alarm_count = FLASH_TIMER_COUNTS;
    receive_alarm.flags.auto_reload_on_alarm = false;
    ESP_ERROR_CHECK(gptimer_set_alarm_action(receive_timer_, &receive_alarm));

    // 発光タイマー
    gptimer_config_t flash_config = {};
    flash_config.clk_src = GPTIMER_CLK_SRC_DEFAULT;
    flash_config.direction = GPTIMER_COUNT_UP;
    flash_config.resolution_hz = TIMER_RESOLUTION_HZ;
    ESP_ERROR_CHECK(gptimer_new_timer(&flash_config, &flash_timer_));

    // 発光タイマー コールバックを登録
    gptimer_event_callbacks_t flash_callback_config = {};
    flash_callback_config.on_alarm = flash_callback;
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(
        flash_timer_, &flash_callback_config, this));

    // 発光タイマー コールバックが発火する条件を設定
    gptimer_alarm_config_t flash_alarm = {};
    flash_alarm.reload_count = 0;
    flash_alarm.alarm_count = INTERVAL_TIMER_COUNTS;
    flash_alarm.flags.auto_reload_on_alarm = true;
    ESP_ERROR_CHECK(gptimer_set_alarm_action(flash_timer_, &flash_alarm));
  }
  virtual ~Photo() {
    ESP_ERROR_CHECK(gptimer_del_timer(flash_timer_));
    ESP_ERROR_CHECK(gptimer_del_timer(receive_timer_));
  }

  bool update() {
    task_ = xTaskGetCurrentTaskHandle();
    esp_err_t receive_enable_err = gptimer_enable(receive_timer_);
    esp_err_t flash_enable_err = gptimer_enable(flash_timer_);
    esp_err_t flash_start_err = gptimer_start(flash_timer_);
    return receive_enable_err == ESP_OK && flash_enable_err == ESP_OK &&
           flash_start_err == ESP_OK;
  }

  bool wait() {
    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
    esp_err_t receive_disable_err = gptimer_disable(receive_timer_);
    esp_err_t flash_disable_err = gptimer_disable(flash_timer_);
    return receive_disable_err == ESP_OK && flash_disable_err == ESP_OK;
  }

  const Result &left90() { return result_[LEFT90_POS]; }
  const Result &left45() { return result_[LEFT45_POS]; }
  const Result &right45() { return result_[RIGHT45_POS]; }
  const Result &right90() { return result_[RIGHT90_POS]; }
};
