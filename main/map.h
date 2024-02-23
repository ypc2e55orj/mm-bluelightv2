#pragma once

// C++
#include <array>
#include <cstdint>
#include <cstdio>
#include <queue>

// Project
#include "parameters.h"

/**
 * 歩数・壁の有無を管理するクラス
 */
class Map {
 public:
  // 自身の向きを定義
  enum Direction : uint8_t {
    DIRECTION_NORTH = 0x00,
    DIRECTION_EAST = 0x01,
    DIRECTION_SOUTH = 0x02,
    DIRECTION_WEST = 0x03,
  };

  // 各方位での壁の状態を示す
  union Walls {
    struct {
      bool north : 1;
      bool east : 1;
      bool south : 1;
      bool west : 1;
      uint8_t _padding : 4;
    } exist;
    struct {
      uint8_t _padding : 4;
      bool north : 1;
      bool east : 1;
      bool south : 1;
      bool west : 1;
    } stepped;
    struct {
      uint8_t exist : 4;
      uint8_t stepped : 4;
    } byte;
  };

  // 迷路での座標
  struct Coord {
    int x;
    int y;
  };

  // 標準出力時の壁の色を定義
  static constexpr auto WALL_COLOR_NOT_EXISTS = "\x1b[37m";
  static constexpr auto WALL_COLOR_EXISTS = "\x1b[37m";
  static constexpr auto WALL_COLOR_UNKNOWN = "\x1b[31m";
  static constexpr auto WALL_COLOR_RESET = "\x1b[0m";

  // コンストラクタ
  explicit Map(const int (&goal_xs)[MAZE_GOAL_SIZE], const int (&goal_ys)[MAZE_GOAL_SIZE]);
  // デストラクタ
  ~Map();

  // 壁情報を初期化
  void initWalls();

  // スタートまでの歩数マップを初期化
  void initStepsToStart();
  // ゴールまでの歩数マップを初期化
  void initStepsToGoal(const int (&goal_xs)[MAZE_GOAL_SIZE], const int (&goal_ys)[MAZE_GOAL_SIZE]);

  // 歩数を作成
  void makeSteps(bool shortest);

  // 自身の位置を取得する
  const Coord &getPos() { return pos_; }
  // 自身の位置を設定する
  void setPos(int x, int y) {
    pos_.x = x;
    pos_.y = y;
  }
  void setPos(Direction dir) {
    dir_ = dir;
    switch (dir) {
      case DIRECTION_NORTH:
        pos_.y++;
        break;
      case DIRECTION_EAST:
        pos_.x++;
        break;
      case DIRECTION_SOUTH:
        pos_.y--;
        break;
      case DIRECTION_WEST:
        pos_.x--;
        break;
    }
  }
  // 自身の方位を回転させる
  void rotateDir() { dir_ = static_cast<Direction>((dir_ + 2) & 0x03); }

  // 自身がゴールしているか取得する
  [[nodiscard]] bool inGoal(const int (&goal_xs)[MAZE_GOAL_SIZE], const int (&goal_ys)[MAZE_GOAL_SIZE]) const {
    for (const auto &y : goal_ys) {
      for (const auto &x : goal_xs) {
        if (pos_.y == y && pos_.x == x) return true;
      }
    }
    return false;
  }
  // 自身がスタート座標にいるか取得する
  [[nodiscard]] bool inStart() const { return pos_.y == 0 && pos_.x == 0; }

  // 壁を設定する
  void setWall(int x, int y, bool frontRight, bool right, bool left, bool frontLeft);
  void setWall(int x, int y, Walls walls);

  // 次に進む方向を取得する
  Direction getNextDir();

  // 迷路を標準出力する
  void print();

 private:
  // 歩数を保持する2次元配列
  std::array<std::array<uint8_t, MAZE_SIZE_X>, MAZE_SIZE_Y> steps_;
  // 壁の有無、訪問済みかを保持する2次元配列
  std::array<std::array<Walls, MAZE_SIZE_X>, MAZE_SIZE_Y> walls_;

  std::queue<Coord> updateQueue_;
  Direction dir_;
  Coord pos_;

  // 歩数マップを初期化
  void initSteps() {
    for (auto &row : steps_) {
      for (auto &step : row) {
        // すべての座標の歩数を最大値に設定
        step = 255;
      }
    }
  }

  // 指定座標が未探索かどうかを取得
  bool isNotVisited(int x, int y) {
    const auto &walls = walls_[y][x];
    return !(walls.stepped.north && walls.stepped.east && walls.stepped.south && walls.stepped.west);
  }

  // 進行方向の優先度を取得する
  int getPriority(int x, int y, Direction dir) {
    int priority = 0;

    if (dir_ == dir) {
      // 進行方向が同じ場合
      priority = 2;
    } else if (std::abs(dir_ - dir) == 2) {
      // 進行方向が逆
      priority = 0;
    } else {
      priority = 1;
    }

    // 未探索の場合、優先度を更に付加
    if (isNotVisited(x, y)) priority += 4;

    return priority;
  }

  // 自身の向きを標準出力する
  static void printPos(Direction dir);

  // 壁を標準出力する
  static void printWall(Direction dir, Walls walls);
};
