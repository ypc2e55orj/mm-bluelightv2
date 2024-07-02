#pragma once

// Project
#include "motion.h"
#include "sensor.h"

/**
 * 目標値設定クラス
 */
class Run {
 public:
  // コンストラクタ
  explicit Run(Motion *motion);
  // デストラクタ
  ~Run();

  // 走行
  void straight(MotionDirection direction, float length, float acceleration, float max_velocity, float end_velocity);
  void turn(float degree, float angular_acceleration, float max_angular_velocity, MotionDirection dir);
  void stop();

 private:
  // 目標生成クラス
  Motion *motion_;
};
