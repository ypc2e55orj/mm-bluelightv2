#pragma once

// C++
#include <memory>

// ESP-IDF
#include <driver/gpio.h>

class Gpio {
 private:
  gpio_num_t num_;

 public:
  explicit Gpio(gpio_num_t num, gpio_mode_t mode, bool enable_pullup,
                bool enable_pulldown)
      : num_(num) {
    // 引数で指定されたピンを初期化
    gpio_config_t config = {};
    config.pin_bit_mask = 1ULL << num;
    config.mode = mode;
    config.pull_up_en =
        enable_pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    config.pull_down_en =
        enable_pulldown ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&config));
  }
  ~Gpio() = default;

  // 出力
  bool set(bool level) {
    esp_err_t set_err = gpio_set_level(num_, level ? 1 : 0);
    return set_err == ESP_OK;
  }

  // 入力
  bool get() { return gpio_get_level(num_) == 1; }
};
