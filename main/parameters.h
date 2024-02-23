#pragma once

// C++
#include <cstdint>
#include <numbers>

enum {
  PARAMETER_WALL_RIGHT90,
  PARAMETER_WALL_RIGHT45,
  PARAMETER_WALL_LEFT90,
  PARAMETER_WALL_LEFT45,
  NUM_PARAMETER_WALL,
};
enum {
  PARAMETER_PID_KP,
  PARAMETER_PID_KI,
  PARAMETER_PID_KD,
  NUM_PARAMETER_PID,
};

/**
 * パラメータの意味は下記:
 * https://rt-net.jp/mobility/archives/16525
 * http://hidejrlab.blog104.fc2.com/blog-entry-1234.html
 */
// トレッド間距離 [mm]
constexpr float TREAD_WIDTH = 33.0f;
// モーターの逆起電圧定数 [mV/rpm]
constexpr float MOTOR_KE = 0.062f;
// ギア比 [spur/pinion]
constexpr float GEAR_RATIO = 38.0f / 9.0f;
// タイヤの直径 [mm]
constexpr float TIRE_DIAMETER = 12.80f;

// 動作停止電圧 [mV]
constexpr int VOLTAGE_LOW_LIMIT = 3200;
// モーター電圧上限 [V]
constexpr float VOLTAGE_MOTOR_LIMIT = 2.0f;

// 最小速度 [m/s]
constexpr float VELOCITY_MIN = 0.1f;
// デフォルト速度 [m/s]
constexpr float VELOCITY_DEFAULT = 0.3f;
// デフォルト加速度 [m/s^2]
constexpr float ACCELERATION_DEFAULT = 1.0f;
// 速度PIDゲイン
constexpr float VELOCITY_PID_GAIN[NUM_PARAMETER_PID] = {6.0f, 0.0f, 0.0f};
// 最小角速度 [rad/s]
constexpr float ANGULAR_VELOCITY_MIN = std::numbers::pi_v<float> / 10.0f;
// デフォルト角速度 [rad/s]
constexpr float ANGULAR_VELOCITY_DEFAULT = std::numbers::pi_v<float>;
// デフォルト角加速度 [rad/s^2]
constexpr float ANGULAR_ACCELERATION_DEFAULT = 20 * std::numbers::pi_v<float>;
// 角速度PIDゲイン
constexpr float ANGULAR_VELOCITY_PID_GAIN[NUM_PARAMETER_PID] = {0.4f, 0.0f, 0.0f};

// 壁センサでの壁有無しきい値 (r90, r45, l45, l90)
constexpr int WALL_THRESHOLD_EXIST[NUM_PARAMETER_WALL] = {0, 0, 0, 0};
// 壁センサの迷路中央基準値 (r90, r45, l45, l90)
constexpr int WALL_REFERENCE_VALUE[NUM_PARAMETER_WALL] = {0, 0, 0, 0};
// 横壁制御PIDゲイン
constexpr float WALL_ADJUST_SIDE_PID_GAIN[NUM_PARAMETER_PID] = {0.0f, 0.0f, 0.0f};

// 迷路の大きさ
constexpr int MAZE_SIZE_X = 32;
constexpr int MAZE_SIZE_Y = 32;
// 迷路のゴール区画の大きさ
constexpr int MAZE_GOAL_SIZE = 2;
// 迷路のゴール座標
constexpr int MAZE_GOAL_X[MAZE_GOAL_SIZE] = {3, 4};
constexpr int MAZE_GOAL_Y[MAZE_GOAL_SIZE] = {3, 4};

// モード選択でモード確定とする壁センサしきい値 (l90, l45, r45, r90)
constexpr int MODE_THRESHOLD_WALL[NUM_PARAMETER_WALL] = {1000, 1000, 1000, 1000};
// モード選択でモード切り替えとする速度 [m/s]
constexpr float MODE_THRESHOLD_SPEED = 0.1f;