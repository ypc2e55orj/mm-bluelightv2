#pragma once

// Project
#include "dri/driver.h"
#include "odometry.h"
#include "queue.h"

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
  // x座標 [mm]
  float x;
  // y座標 [mm]
  float y;
  // バッテリー電圧 [mV]
  int battery_voltage;
  // バッテリー移動平均電圧 [mV]
  int battery_voltage_average;
  // 壁情報
  struct Wall {
    int raw;
    int error;
    bool exist;
  } wall_left90, wall_left45, wall_right45, wall_right90;
};

/**
 * センサ値管理・更新を行うクラス
 */
class Sensor {
 public:
  // 初回センサー読み捨て回数
  static constexpr uint32_t WARM_UP_COUNTS = 10;

  explicit Sensor(Driver *dri);
  ~Sensor();

  // 初期化
  void setup();

  // 更新
  void update();

  // センサ値キューを取得
  rtos::Queue<Sensed> &getSensedQueue() { return sensed_queue_; }

  // リセット
  void reset() { odom_.reset(); }

 private:
  // ドライバ
  Driver *dri_;

  // 内部計算値
  Odometry odom_;

  // 最新のセンサー値を保持するキュー
  rtos::Queue<Sensed> sensed_queue_{1};

  // タイムスタンプ
  int64_t timestamp_{};

  // 壁ADC値を更新
  void updateWallSensor(Sensed &sensed);
};