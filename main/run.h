#pragma once

// Project
#include "motion.h"
#include "sensor.h"

// 走行方向
enum class RunDirection { Right, Left };

/**
 * 目標値設定クラス
 */
class Run {
 public:
  // コンストラクタ
  explicit Run(Driver *driver, Sensor *sensor, Motion *motion);
  // デストラクタ
  ~Run();

  // 走行
  void straight(float length, float acceleration, float max_velocity, float end_velocity);
  void turn(float degree, float angular_acceleration, float max_angular_velocity, RunDirection dir);

 private:
  // ドライバ
  Driver *driver_;

  // センサ
  Sensor *sensor_;

  // 目標生成クラス
  Motion *motion_;
};
