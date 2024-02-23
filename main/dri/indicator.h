#pragma once

// C++
#include <cstring>

// ESP-IDF
#include <driver/rmt_tx.h>

/**
 * 参考:
 * https://github.com/espressif/esp-idf/tree/release/v5.1/examples/peripherals/rmt/led_strip
 */
class Indicator {
 public:
  explicit Indicator(gpio_num_t indicator_num, uint16_t led_counts, uint32_t queue_size, bool with_dma)
      : channel_(nullptr), encoder_(nullptr), buffer_(), led_counts_(led_counts), buffer_size_() {
    // 送信バッファを初期化
    buffer_ = reinterpret_cast<uint8_t *>(
        heap_caps_calloc(WS2812C_COLOR_DEPTH * led_counts, sizeof(uint8_t), MALLOC_CAP_DMA));
    buffer_size_ = WS2812C_COLOR_DEPTH * led_counts * sizeof(uint8_t);
    // RMT送信チャンネルを初期化
    rmt_tx_channel_config_t rmt_config = {};
    rmt_config.gpio_num = indicator_num;
    rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
    rmt_config.resolution_hz = INDICATOR_RESOLUTION_HZ;
    rmt_config.mem_block_symbols = INDICATOR_MEM_BLOCK_SYMBOLS;
    rmt_config.trans_queue_depth = queue_size;
    rmt_config.flags.with_dma = with_dma ? 1 : 0;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&rmt_config, &channel_));

    // エンコーダーを初期化
    uint32_t reset_ticks = INDICATOR_RESOLUTION_HZ / 1'000'000 * 50 / 2;
    encoder_ = reinterpret_cast<IndicatorEncoder *>(heap_caps_calloc(1, sizeof(IndicatorEncoder), MALLOC_CAP_DMA));
    encoder_->base.encode = Indicator::rmt_indicator_encode;
    encoder_->base.del = Indicator::rmt_indicator_delete;
    encoder_->base.reset = Indicator::rmt_indicator_reset;
    encoder_->reset_code.level0 = 0;
    encoder_->reset_code.duration0 = static_cast<uint16_t>(reset_ticks);
    encoder_->reset_code.level1 = 0;
    encoder_->reset_code.duration1 = static_cast<uint16_t>(reset_ticks);
    // bytes_encoderを初期化
    uint32_t bit_ticks = INDICATOR_RESOLUTION_HZ / 1'000'000;
    rmt_bytes_encoder_config_t bytes_encoder_config = {};
    bytes_encoder_config.bit0.level0 = 1;
    bytes_encoder_config.bit0.duration0 = static_cast<uint16_t>(0.3 * bit_ticks);
    bytes_encoder_config.bit0.level1 = 0;
    bytes_encoder_config.bit0.duration1 = static_cast<uint16_t>(0.9 * bit_ticks);
    bytes_encoder_config.bit1.level0 = 1;
    bytes_encoder_config.bit1.duration0 = static_cast<uint16_t>(0.9 * bit_ticks);
    bytes_encoder_config.bit1.level1 = 0;
    bytes_encoder_config.bit1.duration1 = static_cast<uint16_t>(0.3 * bit_ticks);
    bytes_encoder_config.flags.msb_first = 1;
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&bytes_encoder_config, &encoder_->bytes_encoder));
    // copy_encoderを初期化
    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_encoder_config, &encoder_->copy_encoder));
  }
  ~Indicator() {
    rmt_del_channel(channel_);
    if (buffer_) {
      free(buffer_);
      buffer_ = nullptr;
      buffer_size_ = 0;
    }
  }

  [[nodiscard]] uint16_t counts() const { return led_counts_; }

  bool enable() {
    esp_err_t enable_err = rmt_enable(channel_);
    return enable_err == ESP_OK;
  }

  bool disable() {
    esp_err_t disable_err = rmt_disable(channel_);
    return disable_err == ESP_OK;
  }

  void set(size_t pos, uint8_t r, uint8_t g, uint8_t b) {
    buffer_[pos * WS2812C_COLOR_DEPTH] = g;
    buffer_[pos * WS2812C_COLOR_DEPTH + 1] = r;
    buffer_[pos * WS2812C_COLOR_DEPTH + 2] = b;
  }
  void set(size_t pos, uint32_t rgb) { set(pos, (rgb & 0xFF0000) >> 16, (rgb & 0xFF00) >> 8, (rgb & 0xFF)); }
  void clear() { memset(buffer_, 0, buffer_size_); }

  bool update() {
    rmt_transmit_config_t tx_config = {};
    tx_config.loop_count = 0;
    esp_err_t transmit_err = rmt_transmit(channel_, &encoder_->base, buffer_, buffer_size_, &tx_config);
    return transmit_err == ESP_OK;
  }

 private:
  // DMA有効時、DMAのバッファサイズ
  // DMA無効時、チャンネルが専有するメモリブロック(48以上)
  static constexpr size_t INDICATOR_MEM_BLOCK_SYMBOLS = 48;
  // 内部カウンタの精度
  static constexpr uint32_t INDICATOR_RESOLUTION_HZ = 10'000'000;
  // LEDの色(緑、赤、青)
  static constexpr uint32_t WS2812C_COLOR_DEPTH = 3;
  // エンコーダーに渡す構造体
  struct IndicatorEncoder {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    int state;
    rmt_symbol_word_t reset_code;
  };
  using IndicatorEncoderHandle = IndicatorEncoder *;

  // コールバック
  static size_t rmt_indicator_encode(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data,
                                     size_t data_size, rmt_encode_state_t *ret_state) {
    auto indicator_encoder = __containerof(encoder, IndicatorEncoder, base);
    auto bytes_encoder = indicator_encoder->bytes_encoder;
    auto copy_encoder = indicator_encoder->copy_encoder;

    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    int state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;

    switch (indicator_encoder->state) {
      case 0:  // RGBを送信
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, primary_data, data_size, &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
          indicator_encoder->state = 1;  // 次でリセットコードを送信
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
          state |= RMT_ENCODING_MEM_FULL;
          goto err;
        }
        [[fallthrough]];
      case 1:  // リセットコードを送信
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &indicator_encoder->reset_code,
                                                sizeof(indicator_encoder->reset_code), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
          indicator_encoder->state = 0;  // 次でRGBを送信
          state |= RMT_ENCODING_COMPLETE;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
          state |= RMT_ENCODING_MEM_FULL;
          goto err;
        }
    }

  err:
    *ret_state = static_cast<rmt_encode_state_t>(state);
    return encoded_symbols;
  }

  static esp_err_t rmt_indicator_delete(rmt_encoder_t *encoder) {
    auto indicator_encoder = __containerof(encoder, IndicatorEncoder, base);
    rmt_del_encoder(indicator_encoder->bytes_encoder);
    rmt_del_encoder(indicator_encoder->copy_encoder);
    free(indicator_encoder);

    return ESP_OK;
  }

  static esp_err_t rmt_indicator_reset(rmt_encoder_t *encoder) {
    auto indicator_encoder = __containerof(encoder, IndicatorEncoder, base);
    rmt_encoder_reset(indicator_encoder->bytes_encoder);
    rmt_encoder_reset(indicator_encoder->copy_encoder);
    indicator_encoder->state = 0;

    return ESP_OK;
  }

  // 送信チャンネルのハンドラ
  rmt_channel_handle_t channel_;
  // エンコーダーのハンドラ
  IndicatorEncoderHandle encoder_;

  // 送信バッファの大きさ
  uint8_t *buffer_;
  uint16_t led_counts_;
  size_t buffer_size_;
};
