#include <unistd.h>

#include "../../main/map.h"
#include "data/32MM2022HX.inc"

#define DELAY_MS 50

int main(int argc, char **argv) {
  const auto mazeData = _32MM2022HX_maze;
  int goal_x[MAZE_GOAL_SIZE_X] = {14, 15, 16},
      goal_y[MAZE_GOAL_SIZE_Y] = {14, 15, 16};
  Map map(goal_x, goal_y);
  Map::Direction dir = Map::DIRECTION_NORTH;

  map.setPos(0, 0);

  printf("\x1b[2J");
  printf("\x1b[0;0H");
  map.print();

  Map::Walls walls;

  // 探索走行
  while (!map.inGoal(goal_x, goal_y)) {
    // 壁設定
    uint8_t wallsSer = mazeData[map.getPos().y][map.getPos().x];
    walls.north =
        ((wallsSer >> 0) & 0x01) ? Map::WALL_EXISTS : Map::WALL_NOT_EXISTS;
    walls.east =
        ((wallsSer >> 1) & 0x01) ? Map::WALL_EXISTS : Map::WALL_NOT_EXISTS;
    walls.south =
        ((wallsSer >> 2) & 0x01) ? Map::WALL_EXISTS : Map::WALL_NOT_EXISTS;
    walls.west =
        ((wallsSer >> 3) & 0x01) ? Map::WALL_EXISTS : Map::WALL_NOT_EXISTS;

    map.setWall(map.getPos().x, map.getPos().y, walls);
    // 歩数マップを更新
    map.initStepsToGoal(goal_x, goal_y);
    map.makeSteps(false);
    // 次に進んで、自分の位置を更新
    map.setPos(map.getNextDir(false));
    // 出力
    printf("\x1b[0;0H");
    map.print();
    usleep(1000 * DELAY_MS);
  }
  // スタート座標まで戻る
  while (!map.inStart()) {
    // 壁設定
    uint8_t wallsSer = mazeData[map.getPos().y][map.getPos().x];
    walls.north =
        ((wallsSer >> 0) & 0x01) ? Map::WALL_EXISTS : Map::WALL_NOT_EXISTS;
    walls.east =
        ((wallsSer >> 1) & 0x01) ? Map::WALL_EXISTS : Map::WALL_NOT_EXISTS;
    walls.south =
        ((wallsSer >> 2) & 0x01) ? Map::WALL_EXISTS : Map::WALL_NOT_EXISTS;
    walls.west =
        ((wallsSer >> 3) & 0x01) ? Map::WALL_EXISTS : Map::WALL_NOT_EXISTS;

    map.setWall(map.getPos().x, map.getPos().y, walls);
    // 歩数マップを更新
    map.initStepsToStart();
    map.makeSteps(false);
    // 次に進んで、自分の位置を更新
    map.setPos(map.getNextDir(false));
    // 出力
    printf("\x1b[0;0H");
    map.print();
    usleep(1000 * DELAY_MS);
  }
  // 最短走行
}