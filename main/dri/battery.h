#pragma once

// Project
#include "adc.h"
#include "average.h"

class Battery {
 private:
  // バッテリー分圧抵抗に接続されたADC
  Adc adc_;
  // バッテリー電圧を移動平均する
  data::MovingAverage<int, int, 512> average_{};
  // 最終測定値
  int voltage_{};
  // 移動平均した値
  int average_voltage_{};

 public:
  explicit Battery(adc_unit_t unit, adc_channel_t channel)
      : adc_(unit, channel) {}
  ~Battery() = default;

  bool update() {
    adc_.read();
    // 分圧されているため2倍、実測調整で+100
    voltage_ = adc_.to_voltage() * 2 + 100;
    // 電圧の移動平均を取得
    average_voltage_ = average_.update(voltage_);
    // 失敗した場合はAdcクラスでabortするので常にtrue
    return true;
  }

  [[nodiscard]] int voltage() const { return voltage_; }
  [[nodiscard]] int average() const { return average_voltage_; }
};
