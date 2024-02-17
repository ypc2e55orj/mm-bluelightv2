#pragma once

// ESP-IDF
#include <esp_console.h>

class Console {
 private:
  static constexpr auto PROMPT = "mm-bluelight >";
  esp_console_repl_t *repl;

 public:
  explicit Console() : repl(nullptr) {
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = PROMPT;
    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
  }

  void start() { ESP_ERROR_CHECK(esp_console_start_repl(repl)); }
  void stop() { ESP_ERROR_CHECK(repl->del(repl)); }

  void reg(esp_console_cmd_t *cmd) {  // NOLINT
    ESP_ERROR_CHECK(esp_console_cmd_register(cmd));
  }
};
