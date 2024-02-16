#pragma once

// C++
#include <stdexcept>

// ESP-IDF
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_private/adc_private.h>

class Adc {
 private:
  static constexpr auto ATTEN_DB = ADC_ATTEN_DB_11;
  static constexpr auto BITWIDTH = ADC_BITWIDTH_12;

  // 各ユニットのハンドラ
  static adc_oneshot_unit_handle_t unit1_;
  static adc_oneshot_unit_handle_t unit2_;
  // 各ユニットの補正ハンドラ
  static adc_cali_handle_t unit1_cali_;
  static adc_cali_handle_t unit2_cali_;

  // 使用するチャンネル
  adc_channel_t channel_;
  // 使用するユニットのハンドラ
  adc_oneshot_unit_handle_t unit_;
  // 使用するユニットの補正ハンドラ
  adc_cali_handle_t unit_cali_;

  // ADC値
  int raw_;
  // 電圧補正値
  int voltage_;

 public:
  explicit Adc(adc_unit_t unit, adc_channel_t channel)
      : channel_(channel), unit_(nullptr), unit_cali_(nullptr), raw_(0), voltage_(0) {
    adc_oneshot_unit_init_cfg_t init_cfg = {};
    adc_cali_curve_fitting_config_t cali_cfg = {};
    cali_cfg.atten = ATTEN_DB;
    cali_cfg.bitwidth = BITWIDTH;

    // ユニットを初期化 (すでに初期化されている場合は処理しない)
    switch (unit) {
      case ADC_UNIT_1:
        if (!unit1_) [[unlikely]] {
          init_cfg.unit_id = ADC_UNIT_1;
          ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &unit1_));
          cali_cfg.unit_id = ADC_UNIT_1;
          ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_cfg, &unit1_cali_));
        }
        unit_ = unit1_;
        unit_cali_ = unit1_cali_;
        break;
      case ADC_UNIT_2:
        if (!unit2_) [[unlikely]] {
          init_cfg.unit_id = ADC_UNIT_2;
          ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &unit2_));
          cali_cfg.unit_id = ADC_UNIT_2;
          ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_cfg, &unit2_cali_));
        }
        unit_ = unit2_;
        unit_cali_ = unit2_cali_;
        break;
      default:
        assert(false && "Error Adc::AdcImpl(): ADC unit must be specified");
    }

    // チャンネルを初期化
    adc_oneshot_chan_cfg_t chan_cfg = {};
    chan_cfg.atten = ATTEN_DB;
    chan_cfg.bitwidth = BITWIDTH;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(unit_, channel, &chan_cfg));
  }
  ~Adc() = default;

  // ADC値を取得する
  int read() {
    ESP_ERROR_CHECK(adc_oneshot_read(unit_, channel_, &raw_));
    return raw_;
  }
  bool read_isr(int &raw) {
    esp_err_t read_err = adc_oneshot_read_isr(unit_, channel_, &raw_);
    raw = raw_;
    return read_err == ESP_OK;
  }

  // 取得済みのADC値から、電圧値に換算する
  int to_voltage() {
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(unit_cali_, raw_, &voltage_));
    return voltage_;
  }
};
