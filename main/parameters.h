#pragma once

// C++
#include <cstdint>
#include <numbers>

enum {
  PARAMETER_WALL_LEFT90,
  PARAMETER_WALL_LEFT45,
  PARAMETER_WALL_RIGHT45,
  PARAMETER_WALL_RIGHT90,
  NUM_PARAMETER_WALL,
};
enum {
  PARAMETER_PID_KP,
  PARAMETER_PID_KI,
  PARAMETER_PID_KD,
  NUM_PARAMETER_PID,
};

// タイヤの直径 [mm]
constexpr float TIRE_DIAMETER = 12.80f;

// 動作停止電圧 [mV]
constexpr int VOLTAGE_LOW_LIMIT = 3200;
// モーター電圧上限 [V]
constexpr float VOLTAGE_MOTOR_LIMIT = 2.0f;

// デフォルト速度 [m/s]
constexpr float VELOCITY_DEFAULT = 0.3f;
// デフォルト加速度 [m/s^2]
constexpr float ACCELERATION_DEFAULT = 1.0;
// 速度PIDゲイン
constexpr float VELOCITY_PID_GAIN[NUM_PARAMETER_PID] = {10.0f, 0.05f, 0.1f};
// デフォルト角速度 [rad/s]
constexpr float ANGULAR_VELOCITY_DEFAULT = std::numbers::pi_v<float>;
// デフォルト角速度 [rad/s^2]
constexpr float ANGULAR_ACCELERATION_DEFAULT = 20 * std::numbers::pi_v<float>;
// 角速度PIDゲイン
constexpr float ANGULAR_VELOCITY_PID_GAIN[NUM_PARAMETER_PID] = {0.7f, 0.01f, 0.1f};

// 壁センサでの壁有無しきい値 (l90, l45, r45, r90)
constexpr int WALL_THRESHOLD_EXIST[NUM_PARAMETER_WALL] = {0, 0, 0, 0};
// 壁センサの迷路中央基準値 (l90, l45, r45, r90)
constexpr int WALL_REFERENCE_VALUE[NUM_PARAMETER_WALL] = {0, 0, 0, 0};
// 横壁制御PIDゲイン
constexpr float WALL_ADJUST_SIDE_PID_GAIN[NUM_PARAMETER_PID] = {5.0f, 0.0f, 0.0f};

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