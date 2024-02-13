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
  // 迷路での座標
  struct Coord {
    int x;
    int y;
  };
  // 壁の状態を定義
  // 探索走行のときは未探索も含めて確認する
  // 最短走行のときは完全に壁がないことを確認する
  enum Wall : uint8_t {
    WALL_NOT_EXISTS = 0x00,
    WALL_EXISTS = 0x01,
    WALL_NOT_VISITED = 0x02,
    WALL_VIRTUAL = 0x03,
  };
  // 各方位での壁の状態を示す
  union Walls {
    struct {
      Wall north : 2;
      Wall east : 2;
      Wall south : 2;
      Wall west : 2;
    };
    uint8_t byte;
  };

  /**
   * コンストラクタ / デストラクタ
   */
  explicit Map(const int (&goal_xs)[MAZE_GOAL_SIZE_X],
               const int (&goal_ys)[MAZE_GOAL_SIZE_Y])
      : steps_(), walls_(), dir_(), pos_() {
    initWalls();
    initStepsToGoal(goal_xs, goal_ys);
  }
  ~Map() = default;

  // 壁情報を初期化
  void initWalls() {
    // 壁を削除
    for (auto &row : walls_) {
      for (auto &wall : row) {
        wall.north = WALL_NOT_VISITED;
        wall.east = WALL_NOT_VISITED;
        wall.south = WALL_NOT_VISITED;
        wall.west = WALL_NOT_VISITED;
      }
    }
    // 外周の壁を設定(横)
    for (auto x = 0; x < MAZE_SIZE_X; x++) {
      walls_[0][x].south = WALL_EXISTS;
      walls_[MAZE_SIZE_Y - 1][x].north = WALL_EXISTS;
    }
    // 外周の壁を設定(横)
    for (auto y = 0; y < MAZE_SIZE_Y; y++) {
      walls_[y][0].west = WALL_EXISTS;
      walls_[y][MAZE_SIZE_X - 1].east = WALL_EXISTS;
    }
    // スタート座標の右壁
    walls_[0][0].east = WALL_EXISTS;
    walls_[0][1].west = WALL_EXISTS;
  }

  // スタートまでの歩数マップを初期化
  void initStepsToStart() {
    initSteps();
    steps_[0][0] = 0;
    updateQueue_.push({0, 0});
  }
  // ゴールまでの歩数マップを初期化
  void initStepsToGoal(const int (&goal_xs)[MAZE_GOAL_SIZE_X],
                       const int (&goal_ys)[MAZE_GOAL_SIZE_Y]) {
    initSteps();
    // ゴール座標の歩数を最小値に設定
    for (const auto &y : goal_ys) {
      for (const auto &x : goal_xs) {
        steps_[x][y] = 0;
        updateQueue_.push({x, y});
      }
    }
  }
  // 歩数を作成
  void makeSteps(bool shortest) {
    const auto mask = shortest ? WALL_VIRTUAL : WALL_EXISTS;

    while (!updateQueue_.empty()) {
      // 先頭を参照
      const auto &coord = updateQueue_.front();

      /**
       * 隣接区画を確認
       * 探索のときは未探索も壁なしとして扱う
       * 最短のときはすべて壁なしの場合のみ
       */
      const auto &step = steps_[coord.y][coord.x];
      const auto &wall = walls_[coord.y][coord.x];
      // 北
      if (coord.y + 1 < MAZE_SIZE_Y && (wall.north & mask) == 0 &&
          steps_[coord.y + 1][coord.x] == 255) {
        steps_[coord.y + 1][coord.x] = step + 1;
        updateQueue_.push({coord.x, coord.y + 1});
      }
      // 東
      if (coord.x + 1 < MAZE_SIZE_X && (wall.east & mask) == 0 &&
          steps_[coord.y][coord.x + 1] == 255) {
        steps_[coord.y][coord.x + 1] = step + 1;
        updateQueue_.push({coord.x + 1, coord.y});
      }
      // 南
      if (coord.y - 1 > -1 && (wall.south & mask) == 0 &&
          steps_[coord.y - 1][coord.x] == 255) {
        steps_[coord.y - 1][coord.x] = step + 1;
        updateQueue_.push({coord.x, coord.y - 1});
      }
      // 西
      if (coord.x - 1 > -1 && (wall.west & mask) == 0 &&
          steps_[coord.y][coord.x - 1] == 255) {
        steps_[coord.y][coord.x - 1] = step + 1;
        updateQueue_.push({coord.x - 1, coord.y});
      }

      // 削除
      updateQueue_.pop();
    }
  }

  // 自身の位置を取得する
  const Map::Coord &getPos() { return pos_; }
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
  // 自身がゴールしているか取得する
  [[nodiscard]] bool inGoal(const int (&goal_xs)[MAZE_GOAL_SIZE_X],
                            const int (&goal_ys)[MAZE_GOAL_SIZE_Y]) const {
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
  void setWall(int x, int y, bool frontLeft, bool left, bool right,
               bool frontRight) {
    Map::Walls walls{};
    switch (dir_) {
      case DIRECTION_NORTH:
        walls.north =
            frontRight || frontLeft ? Map::WALL_EXISTS : Map::WALL_NOT_EXISTS;
        walls.east = right ? Map::WALL_EXISTS : Map::WALL_NOT_EXISTS;
        walls.west = left ? Map::WALL_EXISTS : Map::WALL_NOT_EXISTS;
        walls.south = Map::WALL_NOT_EXISTS;
        break;
      case DIRECTION_EAST:
        walls.east =
            frontRight || frontLeft ? Map::WALL_EXISTS : Map::WALL_NOT_EXISTS;
        walls.south = right ? Map::WALL_EXISTS : Map::WALL_NOT_EXISTS;
        walls.north = left ? Map::WALL_EXISTS : Map::WALL_NOT_EXISTS;
        walls.west = Map::WALL_NOT_EXISTS;
        break;
      case DIRECTION_SOUTH:
        walls.south =
            frontRight || frontLeft ? Map::WALL_EXISTS : Map::WALL_NOT_EXISTS;
        walls.west = right ? Map::WALL_EXISTS : Map::WALL_NOT_EXISTS;
        walls.east = left ? Map::WALL_EXISTS : Map::WALL_NOT_EXISTS;
        walls.north = Map::WALL_NOT_EXISTS;
        break;
      case DIRECTION_WEST:
        walls.west =
            frontRight || frontLeft ? Map::WALL_EXISTS : Map::WALL_NOT_EXISTS;
        walls.north = right ? Map::WALL_EXISTS : Map::WALL_NOT_EXISTS;
        walls.south = left ? Map::WALL_EXISTS : Map::WALL_NOT_EXISTS;
        walls.east = Map::WALL_NOT_EXISTS;
        break;
    }
    setWall(x, y, walls);
  }
  void setWall(int x, int y, Walls &walls) {
    // 壁を設定
    walls_[y][x].byte = walls.byte;
    // 隣接区画の壁も更新
    if (y + 1 < MAZE_SIZE_Y) {
      // 北 - 南
      walls_[y + 1][x].south = walls.north;
    }
    if (x + 1 < MAZE_SIZE_X) {
      // 東 - 西
      walls_[y][x + 1].west = walls.east;
    }
    if (y - 1 > -1) {
      // 南 - 北
      walls_[y - 1][x].north = walls.south;
    }
    if (x - 1 > -1) {
      // 西 - 東
      walls_[y][x - 1].east = walls.west;
    }
  }

  // 次に進む方向を取得する
  Direction getNextDir(bool shortest) {
    const auto mask = shortest ? WALL_VIRTUAL : WALL_EXISTS;
    Direction dir{};
    uint8_t minStep = 255;
    int priority = 0;

    // 北
    if ((walls_[pos_.y][pos_.x].north & mask) == WALL_NOT_EXISTS &&
        pos_.y + 1 < MAZE_SIZE_Y) {
      int pri = getPriority(pos_.x, pos_.y + 1, DIRECTION_NORTH);
      auto step = steps_[pos_.y + 1][pos_.x];
      if (step < minStep) {
        // 歩数が少ない方を採用
        minStep = step;
        dir = DIRECTION_NORTH;
        priority = pri;
      } else if (step == minStep && priority < pri) {
        // 歩数が同じなら、優先度で判断
        dir = DIRECTION_NORTH;
        priority = pri;
      }
    }
    // 東
    if ((walls_[pos_.y][pos_.x].east & mask) == WALL_NOT_EXISTS &&
        pos_.x + 1 < MAZE_SIZE_X) {
      int pri = getPriority(pos_.x + 1, pos_.y, DIRECTION_EAST);
      auto step = steps_[pos_.y][pos_.x + 1];
      if (step < minStep) {
        minStep = step;
        dir = DIRECTION_EAST;
        priority = pri;
      } else if (step == minStep && priority < pri) {
        dir = DIRECTION_EAST;
        priority = pri;
      }
    }
    // 南
    if ((walls_[pos_.y][pos_.x].south & mask) == WALL_NOT_EXISTS &&
        pos_.y - 1 > -1) {
      int pri = getPriority(pos_.x, pos_.y - 1, DIRECTION_SOUTH);
      auto step = steps_[pos_.y - 1][pos_.x];
      if (step < minStep) {
        minStep = step;
        dir = DIRECTION_SOUTH;
        priority = pri;
      } else if (step == minStep && priority < pri) {
        dir = DIRECTION_SOUTH;
        priority = pri;
      }
    }
    // 西
    if ((walls_[pos_.y][pos_.x].west & mask) == WALL_NOT_EXISTS &&
        pos_.x - 1 > -1) {
      int pri = getPriority(pos_.x - 1, pos_.y, DIRECTION_WEST);
      auto step = steps_[pos_.y][pos_.x - 1];
      if (step < minStep) {
        minStep = step;
        dir = DIRECTION_WEST;
        priority = pri;
      } else if (step == minStep && priority < pri) {
        dir = DIRECTION_WEST;
        priority = pri;
      }
    }

    return dir;
  }

  // 迷路を標準出力する
  void print() {
    constexpr auto MAZE_VERT_INDEX_PADDING = "    ";
    constexpr auto MAZE_VERT_INDEX_FORMAT = " %2d ";
    constexpr auto MAZE_HORIZ_INDEX_PADDING = "    ";
    constexpr auto MAZE_HORIZ_INDEX_FORMAT = " %2d ";

    constexpr auto WALL_COLOR_NOT_EXISTS = "\x1b[37m";
    constexpr auto WALL_COLOR_EXISTS = "\x1b[37m";
    constexpr auto WALL_COLOR_VIRTUAL = "\x1b[33m";
    constexpr auto WALL_COLOR_UNKNOWN = "\x1b[31m";
    constexpr auto WALL_COLOR_RESET = "\x1b[0m";

    printf("%sNot Exists :     %s\n", WALL_COLOR_NOT_EXISTS, WALL_COLOR_RESET);
    printf("%sExists     : --- %s\n", WALL_COLOR_EXISTS, WALL_COLOR_RESET);
    printf("%sVirtual    : --- %s\n", WALL_COLOR_VIRTUAL, WALL_COLOR_RESET);
    printf("%sNot Visited: --- %s\n", WALL_COLOR_UNKNOWN, WALL_COLOR_RESET);

    for (auto y = MAZE_SIZE_Y - 1; y > -1; y--) {
      printf("%s", MAZE_VERT_INDEX_PADDING);
      // 北側の壁を出力
      for (auto x = 0; x < MAZE_SIZE_X; x++) {
        switch (walls_[y][x].north) {
          case WALL_NOT_EXISTS:
            printf("+%s   %s", WALL_COLOR_NOT_EXISTS, WALL_COLOR_RESET);
            break;
          case WALL_EXISTS:
            printf("+%s---%s", WALL_COLOR_EXISTS, WALL_COLOR_RESET);
            break;
          case WALL_VIRTUAL:
            printf("+%s---%s", WALL_COLOR_VIRTUAL, WALL_COLOR_RESET);
            break;
          case WALL_NOT_VISITED:
            printf("+%s---%s", WALL_COLOR_UNKNOWN, WALL_COLOR_RESET);
            break;
        }
      }
      printf("+\n");

      printf(MAZE_VERT_INDEX_FORMAT, y);
      for (auto x = 0; x < MAZE_SIZE_X; x++) {
        // 西側の壁を出力
        switch (walls_[y][x].west) {
          case WALL_NOT_EXISTS:
            printf("%s %s", WALL_COLOR_NOT_EXISTS, WALL_COLOR_RESET);
            break;
          case WALL_EXISTS:
            printf("%s|%s", WALL_COLOR_EXISTS, WALL_COLOR_RESET);
            break;
          case WALL_VIRTUAL:
            printf("%s|%s", WALL_COLOR_VIRTUAL, WALL_COLOR_RESET);
            break;
          case WALL_NOT_VISITED:
            printf("%s|%s", WALL_COLOR_UNKNOWN, WALL_COLOR_RESET);
            break;
        }
        // 歩数を出力
        if (pos_.x == x && pos_.y == y) {
          printf(" x ");
        } else {
          printf("%3d", steps_[y][x]);
        }
      }

      // 東側の壁を出力
      switch (walls_[y][MAZE_SIZE_X - 1].east) {
        case WALL_NOT_EXISTS:
          printf("%s %s", WALL_COLOR_NOT_EXISTS, WALL_COLOR_RESET);
          break;
        case WALL_EXISTS:
          printf("%s|%s", WALL_COLOR_EXISTS, WALL_COLOR_RESET);
          break;
        case WALL_VIRTUAL:
          printf("%s|%s", WALL_COLOR_VIRTUAL, WALL_COLOR_RESET);
          break;
        case WALL_NOT_VISITED:
          printf("%s|%s", WALL_COLOR_UNKNOWN, WALL_COLOR_RESET);
          break;
      }
      printf("\n");
    }

    // 南側の壁を出力
    printf("%s", MAZE_HORIZ_INDEX_PADDING);
    for (auto x = 0; x < MAZE_SIZE_X; x++) {
      switch (walls_[0][x].south) {
        case WALL_NOT_EXISTS:
          printf("+%s   %s", WALL_COLOR_NOT_EXISTS, WALL_COLOR_RESET);
          break;
        case WALL_EXISTS:
          printf("+%s---%s", WALL_COLOR_EXISTS, WALL_COLOR_RESET);
          break;
        case WALL_VIRTUAL:
          printf("+%s---%s", WALL_COLOR_VIRTUAL, WALL_COLOR_RESET);
          break;
        case WALL_NOT_VISITED:
          printf("+%s---%s", WALL_COLOR_UNKNOWN, WALL_COLOR_RESET);
          break;
      }
    }
    printf("+\n");

    // x座標目盛りを出力
    printf("%s", MAZE_HORIZ_INDEX_PADDING);
    for (auto x = 0; x < MAZE_SIZE_X; x++) {
      printf(MAZE_HORIZ_INDEX_FORMAT, x);
    }
    printf("\n");
  }

 private:
  std::array<std::array<uint8_t, MAZE_SIZE_X>, MAZE_SIZE_Y> steps_;
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
    return walls.north == WALL_NOT_VISITED || walls.east == WALL_NOT_VISITED ||
           walls.south == WALL_NOT_VISITED || walls.west == WALL_NOT_VISITED;
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
};