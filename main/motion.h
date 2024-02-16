#pragma once

// C++
#include <utility>

// ESP-IDF
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Project
#include "dri/driver.h"
#include "parameters.h"
#include "pid.h"
#include "rtos.h"
#include "sensor.h"

// 走行パターン
enum class MotionPattern { Stop, Straight, Turn };

// 走行パラメータ
struct MotionParameter {
  // 走行パターン
  MotionPattern pattern;
  // 最大速度 [m/s]
  float max_velocity;
  // 加速度 [m/s^2]
  float acceleration;
  // 最大角速度 [rad/s]
  float max_angular_velocity;
  // 角加速度 [rad/s^2]
  float angular_acceleration;
  // 横壁制御 有効/無効
  bool enable_side_wall_adjust;
};

// 目標値
struct MotionTarget {
  // 目標速度 [m/s]
  float velocity;
  // 目標角速度 [rad/s]
  float angular_velocity;
};

/**
 * 走行モードに応じた目標値生成・モーターへの反映を行うクラス (Pro CPUで動作)
 */
class Motion {
 public:
  // コンストラクタ
  explicit Motion(Driver *dri, Sensor *sensor);
  // デストラクタ
  ~Motion();

  // パラメータキューを取得
  rtos::Queue<MotionParameter> &getParameterQueue() { return param_queue_; }

  // 目標値を取得
  const MotionTarget &getTarget() { return target_; }

  // 更新
  void update();

 private:
  // ドライバ
  Driver *driver_;

  // センサ値計算クラス
  Sensor *sensor_;

  // 走行パラメータを保持するキュー
  rtos::Queue<MotionParameter> param_queue_{1};

  // 目標値を保持する
  MotionTarget target_{};

  // 速度フィードバック
  Pid velo_pid_{VELOCITY_PID_GAIN[PARAMETER_PID_KP], VELOCITY_PID_GAIN[PARAMETER_PID_KI],
                VELOCITY_PID_GAIN[PARAMETER_PID_KD]};
  // 角速度フィードバック
  Pid ang_velo_pid_{ANGULAR_VELOCITY_PID_GAIN[PARAMETER_PID_KP], ANGULAR_VELOCITY_PID_GAIN[PARAMETER_PID_KI],
                    ANGULAR_VELOCITY_PID_GAIN[PARAMETER_PID_KD]};
  // 壁フィードバック
  Pid wall_adj_side_pid_{WALL_ADJUST_SIDE_PID_GAIN[PARAMETER_PID_KP], WALL_ADJUST_SIDE_PID_GAIN[PARAMETER_PID_KI],
                         WALL_ADJUST_SIDE_PID_GAIN[PARAMETER_PID_KD]};

  // 目標値を計算する
  static void calcTarget(MotionParameter &param, MotionTarget &target);

  // 横壁からの目標値を計算する
  bool calcSideWallAdjust(const MotionParameter &param, const Sensed &sensed, MotionTarget &target);

  // 目標速度と現在速度のフィードバック値を計算する
  std::pair<float, float> calcFeedback(const MotionParameter &param, const Sensed &sensed, MotionTarget &target);
};
