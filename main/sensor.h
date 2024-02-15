#pragma once

// Project
#include "dri/driver.h"
#include "odometry.h"

/**
 * センサ値管理・更新を行うクラス
 */
class Sensor {
 public:
  // 初回センサー読み捨て回数
  static constexpr uint32_t WARM_UP_COUNTS = 10;

  // 現在の車体情報を保持する構造体
  struct Sensed {
    // 車体速度 [m/s]
    float velocity;
    // 車体角速度 [rad/s]
    float angular_velocity;
    // 車体角度 [deg]
    float angle;
    // 移動距離 [mm]
    float length;
    // バッテリー電圧 [V]
    float battery_voltage;
  };

  explicit Sensor(Driver *dri);
  ~Sensor();

  // 初期化
  void setup();

  // 更新
  void update();

  // 取得
  const Sensed &get() { return sensed_; }

  // リセット
  void reset() { odom_.reset(); }

 private:
  // ドライバ
  Driver *dri_;

  // 内部計算値
  Odometry odom_;

  // 最新のセンサー値
  Sensed sensed_{};

  // タイムスタンプ
  int64_t timestamp_{};
};