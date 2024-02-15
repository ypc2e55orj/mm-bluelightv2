#pragma once

// C++
#include <cmath>
#include <stdexcept>

// ESP-IDF
#include <driver/rmt_tx.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

[[maybe_unused]] static constexpr uint32_t C4 = 261;
[[maybe_unused]] static constexpr uint32_t Cs4 = 277;
[[maybe_unused]] static constexpr uint32_t D4 = 293;
[[maybe_unused]] static constexpr uint32_t Ds4 = 311;
[[maybe_unused]] static constexpr uint32_t E4 = 329;
[[maybe_unused]] static constexpr uint32_t F4 = 349;
[[maybe_unused]] static constexpr uint32_t Fs4 = 369;
[[maybe_unused]] static constexpr uint32_t G4 = 391;
[[maybe_unused]] static constexpr uint32_t Gs4 = 415;
[[maybe_unused]] static constexpr uint32_t A4 = 440;
[[maybe_unused]] static constexpr uint32_t As4 = 466;
[[maybe_unused]] static constexpr uint32_t B4 = 493;
[[maybe_unused]] static constexpr uint32_t C5 = 523;
[[maybe_unused]] static constexpr uint32_t Cs5 = 554;
[[maybe_unused]] static constexpr uint32_t D5 = 587;
[[maybe_unused]] static constexpr uint32_t Ds5 = 622;
[[maybe_unused]] static constexpr uint32_t E5 = 659;
[[maybe_unused]] static constexpr uint32_t F5 = 698;
[[maybe_unused]] static constexpr uint32_t Fs5 = 739;
[[maybe_unused]] static constexpr uint32_t G5 = 783;
[[maybe_unused]] static constexpr uint32_t Gs5 = 830;
[[maybe_unused]] static constexpr uint32_t A5 = 880;
[[maybe_unused]] static constexpr uint32_t As5 = 932;
[[maybe_unused]] static constexpr uint32_t B5 = 987;
[[maybe_unused]] static constexpr uint32_t C6 = 1046;
[[maybe_unused]] static constexpr uint32_t Cs6 = 1108;
[[maybe_unused]] static constexpr uint32_t D6 = 1174;
[[maybe_unused]] static constexpr uint32_t Ds6 = 1244;
[[maybe_unused]] static constexpr uint32_t E6 = 1318;
[[maybe_unused]] static constexpr uint32_t F6 = 1396;
[[maybe_unused]] static constexpr uint32_t Fs6 = 1479;
[[maybe_unused]] static constexpr uint32_t G6 = 1567;
[[maybe_unused]] static constexpr uint32_t Gs6 = 1661;
[[maybe_unused]] static constexpr uint32_t A6 = 1760;
[[maybe_unused]] static constexpr uint32_t As6 = 1864;
[[maybe_unused]] static constexpr uint32_t B6 = 1975;
[[maybe_unused]] static constexpr uint32_t C7 = 2093;
[[maybe_unused]] static constexpr uint32_t Cs7 = 2217;
[[maybe_unused]] static constexpr uint32_t D7 = 2349;
[[maybe_unused]] static constexpr uint32_t Ds7 = 2489;
[[maybe_unused]] static constexpr uint32_t E7 = 2637;
[[maybe_unused]] static constexpr uint32_t F7 = 2793;
[[maybe_unused]] static constexpr uint32_t Fs7 = 2959;
[[maybe_unused]] static constexpr uint32_t G7 = 3135;
[[maybe_unused]] static constexpr uint32_t Gs7 = 3322;
[[maybe_unused]] static constexpr uint32_t A7 = 3520;
[[maybe_unused]] static constexpr uint32_t As7 = 3729;
[[maybe_unused]] static constexpr uint32_t B7 = 3951;
[[maybe_unused]] static constexpr uint32_t C8 = 4186;

// 単音定義
struct Note {
  // 周波数
  uint32_t frequency;  // [Hz]
  // 長さ
  uint32_t duration;  // [ms]
};

/**
 * 参考:
 * https://github.com/espressif/esp-idf/tree/release/v5.1/examples/peripherals/rmt/musical_buzzer
 */
class Buzzer {
 private:
  // DMA有効時、DMAのバッファサイズ
  // DMA無効時、チャンネルが専有するメモリブロック(48以上)
  static constexpr size_t BUZZER_MEM_BLOCK_SYMBOLS = 64;
  // 内部カウンタの精度
  static constexpr uint32_t BUZZER_RESOLUTION_HZ = 1'000'000;
  // エンコーダーに渡す構造体
  struct BuzzerEncoder {
    rmt_encoder_t base;
    rmt_encoder_t *copy_encoder;
    uint32_t resolution;
  };
  using BuzzerEncoderHandle = BuzzerEncoder *;

  // copy_encoderをハンドルするコールバック
  static size_t rmt_buzzer_encode(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data,
                                  size_t, rmt_encode_state_t *ret_state) {
    auto buzzer_encoder = __containerof(encoder, BuzzerEncoder, base);
    auto copy_encoder = buzzer_encoder->copy_encoder;
    auto note = reinterpret_cast<const Note *>(primary_data);
    auto duration = buzzer_encoder->resolution / note->frequency / 2;
    rmt_symbol_word_t symbol = {};
    symbol.level0 = 0;
    symbol.duration0 = static_cast<uint16_t>(duration);
    symbol.level1 = 1;
    symbol.duration1 = static_cast<uint16_t>(duration);

    return copy_encoder->encode(copy_encoder, channel, &symbol, sizeof(rmt_symbol_word_t), ret_state);
  }

  static esp_err_t rmt_buzzer_delete(rmt_encoder_t *encoder) {
    auto buzzer_encoder = __containerof(encoder, BuzzerEncoder, base);
    rmt_del_encoder(buzzer_encoder->copy_encoder);
    free(buzzer_encoder);

    return ESP_OK;
  }

  static esp_err_t rmt_buzzer_reset(rmt_encoder_t *encoder) {
    auto buzzer_encoder = __containerof(encoder, BuzzerEncoder, base);
    rmt_encoder_reset(buzzer_encoder->copy_encoder);

    return ESP_OK;
  }

  // 送信チャンネルのハンドラ
  rmt_channel_handle_t channel_;
  // エンコーダーのハンドラ
  BuzzerEncoderHandle encoder_;

 public:
  explicit Buzzer(gpio_num_t buzzer_num, uint32_t queue_size, bool with_dma) : channel_(nullptr), encoder_(nullptr) {
    // RMT送信チャンネルを初期化
    rmt_tx_channel_config_t rmt_config = {};
    rmt_config.gpio_num = buzzer_num;
    rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
    rmt_config.resolution_hz = BUZZER_RESOLUTION_HZ;
    rmt_config.mem_block_symbols = BUZZER_MEM_BLOCK_SYMBOLS;
    rmt_config.trans_queue_depth = queue_size;
    rmt_config.flags.with_dma = with_dma ? 1 : 0;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&rmt_config, &channel_));

    // ブザー用エンコーダーを初期化
    encoder_ = reinterpret_cast<BuzzerEncoder *>(heap_caps_calloc(1, sizeof(BuzzerEncoder), MALLOC_CAP_DMA));
    encoder_->base.encode = Buzzer::rmt_buzzer_encode;
    encoder_->base.del = Buzzer::rmt_buzzer_delete;
    encoder_->base.reset = Buzzer::rmt_buzzer_reset;
    encoder_->resolution = BUZZER_RESOLUTION_HZ;
    // copy_encoderを初期化
    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_encoder_config, &encoder_->copy_encoder));
  }

  ~Buzzer() { rmt_del_channel(channel_); }

  bool enable() {
    esp_err_t enable_err = rmt_enable(channel_);
    return enable_err == ESP_OK;
  }

  bool disable() {
    esp_err_t disable_err = rmt_disable(channel_);
    return disable_err == ESP_OK;
  }

  bool tone(uint32_t frequency, uint32_t duration_ms) {
    Note note{frequency, duration_ms};
    rmt_transmit_config_t tx_config = {};
    tx_config.loop_count = static_cast<int>(note.duration * note.frequency / 1000);
    esp_err_t transmit_err = rmt_transmit(channel_, &encoder_->base, &note, sizeof(Note), &tx_config);
    esp_err_t wait_err = rmt_tx_wait_all_done(channel_, -1);
    return transmit_err == ESP_OK && wait_err == ESP_OK;
  }
};
