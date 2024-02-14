#include <unistd.h>

#include "../../main/map.h"
#include "data/32MM2022HX.inc"

#define DELAY_MS 50

int main(int argc, char **argv) {
  const auto mazeData = _32MM2022HX_maze;
  int goal_x[MAZE_GOAL_SIZE_X] = {14, 15, 16},
      goal_y[MAZE_GOAL_SIZE_Y] = {14, 15, 16};
  Map map(goal_x, goal_y);

  // 自己位置を原点に設定
  map.setPos(0, 0);

  Map::Walls walls;
  // 探索走行
  while (!map.inGoal(goal_x, goal_y)) {
    // 壁設定
    walls.byte.exist = mazeData[map.getPos().y][map.getPos().x];
    // 壁を設定
    map.setWall(map.getPos().x, map.getPos().y, walls);
    // 出力
    printf("\x1b[0;0H");
    map.print();
    usleep(1000 * DELAY_MS);
    // 歩数マップを更新
    map.initStepsToGoal(goal_x, goal_y);
    map.makeSteps(false);
    // 次に進んで、自分の位置を更新
    map.setPos(map.getNextDir());
  }
  // スタート座標まで戻る
  while (!map.inStart()) {
    // 壁設定
    walls.byte.exist = mazeData[map.getPos().y][map.getPos().x];
    // 壁を設定
    map.setWall(map.getPos().x, map.getPos().y, walls);
    // 出力
    printf("\x1b[0;0H");
    map.print();
    usleep(1000 * DELAY_MS);
    // 歩数マップを更新
    map.initStepsToStart();
    map.makeSteps(false);
    // 次に進んで、自分の位置を更新
    map.setPos(map.getNextDir());
  }
  map.rotateDir();
  // 最短走行
  while (!map.inGoal(goal_x, goal_y)) {
    // 出力
    printf("\x1b[0;0H");
    map.print();
    usleep(1000 * DELAY_MS);
    // 歩数マップを更新
    map.initStepsToGoal(goal_x, goal_y);
    map.makeSteps(true);
    // 次に進んで、自分の位置を更新
    map.setPos(map.getNextDir());
  }
}