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
enum class MotionPattern { Free, Stop, Straight, Turn };
// 回転方向
enum class MotionDirection { Forward, Back, Right, Left };

// モーションタスクに与える走行パラメータ
struct MotionParameter {
  // 走行パターン
  MotionPattern pattern;
  // 走行方向
  MotionDirection direction;
  // 加速度 [m/s^2]
  float acceleration;
  // 開始速度 [m/s]
  float start_velocity;
  // 最大速度 [m/s]
  float max_velocity;
  // 終了速度 [m/s]
  float end_velocity;
  // 走行距離 [mm]
  float length;
  // 角加速度 [rad/s^2]
  float angular_acceleration;
  // 開始角速度 [rad/s]
  float start_angular_velocity;
  // 最大角速度 [rad/s]
  float max_angular_velocity;
  // 終了角速度 [rad/s]
  float end_angular_velocity;
  // 角度 [deg]
  float angle;
  // 横壁制御 有効/無効
  bool enable_side_wall_adjust;
};

// モーションタスク内で生成されて返される値
struct MotionTarget {
  // 加速度 [m/s^2]
  float acceleration;
  // 目標速度 [m/s]
  float velocity;
  // 最大速度 [m/s]
  float max_velocity;
  // 各加速度 [rad/s^2]
  float angular_acceleration;
  // 目標角速度 [rad/s]
  float angular_velocity;
  // 最大角速度 [rad/s]
  float max_angular_velocity;
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

  // パラメータを取得 (書き込み用)
  rtos::Queue<MotionParameter> &getParameterQueue() { return param_queue_; }
  // パラメータを取得 (読み込み用)
  const MotionParameter &getParameter() { return param_; }

  // 目標値を取得
  const MotionTarget &getTarget() { return target_; }

  // 更新
  void update();

  // 動作完了待ち
  void wait();

 private:
  // ドライバ
  Driver *driver_;

  // センサ値計算クラス
  Sensor *sensor_;

  // 完了通知先タスク
  TaskHandle_t handle_notify_{};

  // 走行パラメータを保持するキュー
  rtos::Queue<MotionParameter> param_queue_{1};
  MotionParameter param_{};

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

  // 走行パターンに応じて現在のパラメータを設定
  static bool setPatternParam(const Sensed &sensed, const MotionParameter &param, MotionTarget &target);
  // 直線
  static bool straight(const Sensed &sensed, const MotionParameter &param, MotionTarget &target);
  // 旋回
  static bool turn(const Sensed &sensed, const MotionParameter &param, MotionTarget &target);

  // 目標値を計算する
  static void calcTargetVelocity(MotionPattern pattern, MotionTarget &target);

  // 横壁からの目標値を計算する
  bool adjustSideWall(const Sensed &sensed, const MotionParameter &param, MotionTarget &target);

  // 目標速度と現在速度のフィードバック値を計算する
  std::pair<float, float> calcMotorVoltage(const Sensed &sensed, const MotionParameter &param, MotionTarget &target);
};
