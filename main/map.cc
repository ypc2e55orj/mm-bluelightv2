#include "map.h"

// C++
#include <iomanip>

// コンストラクタ
Map::Map(const int (&goal_xs)[MAZE_GOAL_SIZE], const int (&goal_ys)[MAZE_GOAL_SIZE])
    : steps_(), walls_(), dir_(), pos_() {
  initWalls();
  initStepsToGoal(goal_xs, goal_ys);
}
// デストラクタ
Map::~Map() = default;

// 壁情報を初期化
void Map::initWalls() {
  // 壁を削除
  for (auto &row : walls_) {
    for (auto &wall : row) {
      wall.byte.stepped = 0x00;
      wall.byte.exist = 0x00;
    }
  }
  // 外周の壁を設定(横)
  for (auto x = 0; x < MAZE_SIZE_X; x++) {
    walls_[0][x].exist.south = true;
    walls_[0][x].stepped.south = true;
    walls_[MAZE_SIZE_Y - 1][x].exist.north = true;
    walls_[MAZE_SIZE_Y - 1][x].stepped.north = true;
  }
  // 外周の壁を設定(横)
  for (auto y = 0; y < MAZE_SIZE_Y; y++) {
    walls_[y][0].exist.west = true;
    walls_[y][0].stepped.west = true;
    walls_[y][MAZE_SIZE_X - 1].exist.east = true;
    walls_[y][MAZE_SIZE_X - 1].stepped.east = true;
  }
  // スタート座標の右壁
  walls_[0][0].exist.east = true;
  walls_[0][0].stepped.east = true;
  walls_[0][1].exist.west = true;
  walls_[0][1].stepped.west = true;
}

// スタートまでの歩数マップを初期化
void Map::initStepsToStart() {
  initSteps();
  steps_[0][0] = 0;
  updateQueue_.push({0, 0});
}
// ゴールまでの歩数マップを初期化
void Map::initStepsToGoal(const int (&goal_xs)[MAZE_GOAL_SIZE], const int (&goal_ys)[MAZE_GOAL_SIZE]) {
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
void Map::makeSteps(bool shortest) {
  const auto visited_mask = shortest ? 0x0F : 0x00;

  while (!updateQueue_.empty()) {
    // 先頭を参照
    const auto &coord = updateQueue_.front();

    /**
     * 隣接区画を確認
     * 探索のときは未探索も壁なしとして扱う
     * 最短のときはすべて壁なしの場合のみ
     */
    const auto &step = steps_[coord.y][coord.x];
    const auto &walls = walls_[coord.y][coord.x];
    if ((walls.byte.stepped & visited_mask) == visited_mask) {
      // 北
      if (coord.y + 1 < MAZE_SIZE_Y && steps_[coord.y + 1][coord.x] == 255 && !walls.exist.north) {
        steps_[coord.y + 1][coord.x] = step + 1;
        updateQueue_.push({coord.x, coord.y + 1});
      }
      // 東
      if (coord.x + 1 < MAZE_SIZE_X && steps_[coord.y][coord.x + 1] == 255 && !walls.exist.east) {
        steps_[coord.y][coord.x + 1] = step + 1;
        updateQueue_.push({coord.x + 1, coord.y});
      }
      // 南
      if (coord.y - 1 > -1 && steps_[coord.y - 1][coord.x] == 255 && !walls.exist.south) {
        steps_[coord.y - 1][coord.x] = step + 1;
        updateQueue_.push({coord.x, coord.y - 1});
      }
      // 西
      if (coord.x - 1 > -1 && steps_[coord.y][coord.x - 1] == 255 && !walls.exist.west) {
        steps_[coord.y][coord.x - 1] = step + 1;
        updateQueue_.push({coord.x - 1, coord.y});
      }
    }

    // 削除
    updateQueue_.pop();
  }
}

// 壁を設定する
void Map::setWall(int x, int y, bool frontRight, bool right, bool left, bool frontLeft) {
  Walls walls{};
  switch (dir_) {
    case DIRECTION_NORTH:
      walls.exist.north = frontRight || frontLeft;
      walls.exist.east = right;
      walls.exist.west = left;
      walls.exist.south = false;
      break;
    case DIRECTION_EAST:
      walls.exist.east = frontRight || frontLeft;
      walls.exist.south = right;
      walls.exist.north = left;
      walls.exist.west = false;
      break;
    case DIRECTION_SOUTH:
      walls.exist.south = frontRight || frontLeft;
      walls.exist.west = right;
      walls.exist.east = left;
      walls.exist.north = false;
      break;
    case DIRECTION_WEST:
      walls.exist.west = frontRight || frontLeft;
      walls.exist.north = right;
      walls.exist.south = left;
      walls.exist.east = false;
      break;
  }
  setWall(x, y, walls);
}
void Map::setWall(int x, int y, Walls walls) {
  // 壁を設定
  walls_[y][x].byte.exist = walls.byte.exist;
  walls_[y][x].byte.stepped = 0x0F;
  // 隣接区画の壁も更新
  if (y + 1 < MAZE_SIZE_Y) {
    // 北 - 南
    walls_[y + 1][x].exist.south = walls.exist.north;
    walls_[y + 1][x].stepped.south = true;
  }
  if (x + 1 < MAZE_SIZE_X) {
    // 東 - 西
    walls_[y][x + 1].exist.west = walls.exist.east;
    walls_[y][x + 1].stepped.west = true;
  }
  if (y - 1 > -1) {
    // 南 - 北
    walls_[y - 1][x].exist.north = walls.exist.south;
    walls_[y - 1][x].stepped.north = true;
  }
  if (x - 1 > -1) {
    // 西 - 東
    walls_[y][x - 1].exist.east = walls.exist.west;
    walls_[y][x - 1].stepped.east = true;
  }
}

// 次に進む方向を取得する
Map::Direction Map::getNextDir() {
  Direction dir{};
  uint8_t minStep = 255;
  int priority = 0;

  // 北
  if (!walls_[pos_.y][pos_.x].exist.north && pos_.y + 1 < MAZE_SIZE_Y) {
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
  if (!walls_[pos_.y][pos_.x].exist.east && pos_.x + 1 < MAZE_SIZE_X) {
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
  if (!walls_[pos_.y][pos_.x].exist.south && pos_.y - 1 > -1) {
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
  if (!walls_[pos_.y][pos_.x].exist.west && pos_.x - 1 > -1) {
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

// 自身の向きをストリームに出力する
void Map::outputPos(std::ostream &os, Map::Direction dir) {
  os << "\x1b[34m";
  switch (dir) {
    case DIRECTION_NORTH:
      os << " ^ ";
      break;
    case DIRECTION_EAST:
      os << " > ";
      break;
    case DIRECTION_SOUTH:
      os << " v ";
      break;
    case DIRECTION_WEST:
      os << " < ";
      break;
  }
  os << "\x1b[0m";
}

// 壁をストリームに出力する
void Map::outputWall(std::ostream &os, Map::Direction dir, Map::Walls walls) {
  bool is_exist = (walls.byte.exist & (1 << dir)) != 0x00;
  bool is_stepped = (walls.byte.stepped & (1 << dir)) != 0x00;
  switch (dir) {
    case DIRECTION_NORTH:
    case DIRECTION_SOUTH:
      if (is_stepped) {
        if (is_exist) {
          // 壁あり
          os << "+" << WALL_COLOR_EXISTS << "---" << WALL_COLOR_RESET << "+";
        } else {
          // 壁なし
          os << "+" << WALL_COLOR_NOT_EXISTS << "   " << WALL_COLOR_RESET << "+";
        }
      } else {
        // 不明
        os << "+" << WALL_COLOR_UNKNOWN << "   " << WALL_COLOR_RESET << "+";
      }
      break;

    case DIRECTION_EAST:
    case DIRECTION_WEST:
      if (is_stepped) {
        if (is_exist) {
          // 壁あり
          os << WALL_COLOR_EXISTS << "|" << WALL_COLOR_RESET;
        } else {
          // 壁なし
          os << WALL_COLOR_NOT_EXISTS << " " << WALL_COLOR_RESET;
        }
      } else {
        // 不明
        os << WALL_COLOR_UNKNOWN << "|" << WALL_COLOR_RESET;
      }
      break;
  }
}

// 迷路をストリームに出力する
std::ostream &operator<<(std::ostream &os, const Map &map) {
  for (auto y = MAZE_SIZE_Y - 1; y > -1; y--) {
    os << map.MAZE_VERT_INDEX_PADDING;
    // 北側の壁を出力
    for (auto x = 0; x < MAZE_SIZE_X; x++) {
      map.outputWall(os, map.DIRECTION_NORTH, map.walls_[y][x]);
    }
    os << "+\n";

    os << std::setw(4) << y;
    for (auto x = 0; x < MAZE_SIZE_X; x++) {
      // 西側の壁を出力
      map.outputWall(os, map.DIRECTION_WEST, map.walls_[y][x]);
      // 歩数を出力
      if (map.pos_.x == x && map.pos_.y == y) {
        map.outputPos(os, map.dir_);
      } else {
        os << std::setw(3) << map.steps_[y][x];
      }
    }

    // 東側の壁を出力
    map.outputWall(os, map.DIRECTION_EAST, map.walls_[y][MAZE_SIZE_X - 1]);
    os << "\n";
  }

  os << map.MAZE_HORIZ_INDEX_PADDING;
  // 南側の壁を出力
  for (auto x = 0; x < MAZE_SIZE_X; x++) {
    // 南側の壁を出力
    map.outputWall(os, map.DIRECTION_SOUTH, map.walls_[0][x]);
  }
  os << "+\n";

  os << map.MAZE_HORIZ_INDEX_PADDING;
  // x座標目盛りを出力
  for (auto x = 0; x < MAZE_SIZE_X; x++) {
    os << std::setw(4) << x;
  }
  os << "\n";

  return os;
}
