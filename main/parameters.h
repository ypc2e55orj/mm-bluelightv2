#pragma once

// C++
#include <cstdint>
#include <numbers>

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
constexpr float VELOCITY_PID_GAIN[3] = {10.0f, 0.05f, 0.1f};
// デフォルト角速度 [rad/s]
constexpr float ANGULAR_VELOCITY_DEFAULT = std::numbers::pi_v<float>;
// デフォルト角速度 [rad/s^2]
constexpr float ANGULAR_ACCELERATION_DEFAULT = 20 * std::numbers::pi_v<float>;
// 角速度PIDゲイン
constexpr float ANGULAR_VELOCITY_PID_GAIN[3] = {0.7f, 0.01f, 0.1f};

// 壁センサでの壁有無しきい値
constexpr int WALL_THRESHOLD_EXIST[4] = {0, 0, 0, 0};
// 壁センサでの基準ADC値
constexpr int WALL_REFERENCE_VALUE[4] = {0, 0, 0, 0};
// 横壁制御の強さ
constexpr float WALL_SIDE_ADJUST_PID_GAIN[3] = {5.0f, 0.0f, 0.0f};
// 前壁制御の強さ
constexpr float WALL_FRONT_ADJUST_PID_GAIN[3] = {5.0f, 0.0f, 0.0f};

// 迷路の大きさ
constexpr int MAZE_SIZE_X = 32;
constexpr int MAZE_SIZE_Y = 32;
// 迷路のゴール区画の大きさ
constexpr int MAZE_GOAL_SIZE_X = 2;
constexpr int MAZE_GOAL_SIZE_Y = 2;
// 迷路のゴール座標
constexpr int MAZE_GOAL_X[MAZE_GOAL_SIZE_X] = {3, 4};
constexpr int MAZE_GOAL_Y[MAZE_GOAL_SIZE_Y] = {3, 4};
