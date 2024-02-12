#pragma once

// ESP-IDF
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_intr_alloc.h>

// Project
#include "spi.h"

class Encoder {
 private:
  Spi &spi_;
  int index_;
  uint8_t *tx_buffer_;
  uint8_t *rx_buffer_;
  uint16_t raw_;

  // 送受信バッファサイズ
  static constexpr size_t BUFFER_SIZE = 2;

  // レジスタ
  static constexpr uint16_t REG_MASTER_RESET = 0x33A5;
  static constexpr uint16_t REG_ANGULAR_DATA = 0x3FFF;

  // 送信データにパリティを付与
  static constexpr uint16_t command_frame(uint16_t reg, bool is_reading) {
    reg = (reg << 1) | (is_reading ? 0x8000 : 0x0000);
    return (reg | __builtin_parity(reg));
  }
  static bool verify_angle(uint16_t res) {
    bool parity_ok = (res & 0x0001) == __builtin_parity(res >> 1);
    bool command_ok = (res & 0x0002) == 0;
    /*
    bool alarm_lo_ok = (res & 0x8000) == 0;
    bool alarm_hi_ok = (res & 0x4000) == 0;
    return parity_ok && command_ok & alarm_lo_ok & alarm_hi_ok;
    */
    return parity_ok && command_ok;
  }

 public:
  explicit Encoder(Spi &spi, gpio_num_t spics_io_num) : spi_(spi), raw_(0) {
    // 転送用バッファを確保
    tx_buffer_ = reinterpret_cast<uint8_t *>(
        heap_caps_calloc(BUFFER_SIZE, sizeof(uint8_t), MALLOC_CAP_DMA));
    rx_buffer_ = reinterpret_cast<uint8_t *>(
        heap_caps_calloc(BUFFER_SIZE, sizeof(uint8_t), MALLOC_CAP_DMA));

    // デバイスを追加
    index_ = spi_.add(0, 0, 1, SPI_MASTER_FREQ_10M, spics_io_num, 1);
    auto trans = spi_.transaction(index_);
    // リセット
    trans->flags = 0;
    trans->tx_buffer = tx_buffer_;
    trans->rx_buffer = rx_buffer_;
    trans->length = 16;
    tx_buffer_[0] = command_frame(REG_MASTER_RESET, false) >> 8;
    tx_buffer_[1] = command_frame(REG_MASTER_RESET, false) & 0xFF;
    spi_.transmit(index_);

    // 角度を取得
    tx_buffer_[0] = command_frame(REG_ANGULAR_DATA, true) >> 8;
    tx_buffer_[1] = command_frame(REG_ANGULAR_DATA, true) & 0xFF;
    spi_.transmit(index_);
  }
  ~Encoder() {
    free(tx_buffer_);
    free(rx_buffer_);
  }

  bool update() {
    bool ret = spi_.transmit(index_);
    uint16_t res = rx_buffer_[0] << 8 | rx_buffer_[1];
    if (verify_angle(res)) raw_ = (res >> 2) & 0x3FF;

    return ret;
  }

  [[nodiscard]] uint16_t raw() const { return raw_; }
};
