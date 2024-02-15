#pragma once

// Project
#include "dri/driver.h"
#include "parameters.h"
#include "pid.h"
#include "queue.h"
#include "sensor.h"

// 走行モード
enum class RunMode { Search, Shortest };

// 走行パターン
enum class RunPattern { Stop, Straight, PivotTurnLeft, PivotTurnRight, TurnLeft, TurnRight, FrontAdjust };

// 走行パラメータ
struct RunParameter {
  // 走行モード
  RunMode mode;
  // 走行パターン
  RunPattern pattern;
  // 最大速度 [m/s]
  float max_velocity;
  // 加速度 [m/s^2]
  float acceleration;
  // 最大距離 [mm]
  float max_length;
  // 最大角速度 [rad/s]
  float max_angular_velocity;
  // 角加速度 [rad/s^2]
  float angular_acceleration;
  // 最大角度 [deg]
  float max_angle;
};

// 目標値
struct RunTarget {
  // 目標速度 [m/s]
  float velocity;
  // 目標距離 [mm]
  float length;
  // 目標角速度 [rad/s]
  float angular_velocity;
  // 目標角度 [deg]
  float angle;
};

/**
 * 走行モードに応じた目標値生成・モーターへの反映を行うクラス (Pro CPUで動作)
 */
class Motion {
 public:
  // コンストラクタ
  explicit Motion(Driver *dri, rtos::Queue<Sensed> &sensed_queue);
  // デストラクタ
  ~Motion();

  // パラメータキューを取得
  rtos::Queue<RunParameter> &getParameterQueue() { return param_queue_; }

  // 目標値キューを取得
  rtos::Queue<RunTarget> &getTargetQueue() { return target_queue_; }

  // 更新
  void update();

 private:
  // ドライバ
  Driver *dri_;

  // センサ値を取得するキュー
  rtos::Queue<Sensed> &sensed_queue_;

  // 走行パラメータを保持するキュー
  rtos::Queue<RunParameter> param_queue_{1};

  // 目標値を保持するキュー
  rtos::Queue<RunTarget> target_queue_{1};

  // 速度フィードバック
  Pid velo_pid_{VELOCITY_PID_GAIN[0], VELOCITY_PID_GAIN[1], VELOCITY_PID_GAIN[2]};
  // 角速度フィードバック
  Pid ang_velo_pid_{ANGULAR_VELOCITY_PID_GAIN[0], ANGULAR_VELOCITY_PID_GAIN[1], ANGULAR_VELOCITY_PID_GAIN[2]};

  // 目標値を計算する
  static void calcTarget(RunParameter &param, RunTarget &target);

  // 横壁からの目標値を計算する
  static void calcSideWallAdjust(RunParameter &param, RunTarget &target);
};

/**
 * 走行パターン・目標追従待ちクラス (App CPUで動作)
 */
class Run {
 public:
  explicit Run();

 private:
};
