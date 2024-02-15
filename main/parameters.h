#pragma once

// C++
#include <cstdint>

// タイヤの直径 [mm]
constexpr float TIRE_DIAMETER = 12.80f;

// 速度PIDゲイン
constexpr float PID_GAIN_VELOCITY[3] = {};
// 角速度PIDゲイン
constexpr float PID_GAIN_ANGULAR_VELOCITY[3] = {};

// 迷路の大きさ
constexpr int MAZE_SIZE_X = 32;
constexpr int MAZE_SIZE_Y = 32;
// 迷路のゴール区画の大きさ
constexpr int MAZE_GOAL_SIZE_X = 3;
constexpr int MAZE_GOAL_SIZE_Y = 3;