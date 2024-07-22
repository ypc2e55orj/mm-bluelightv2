// C++
#include <array>
#include <cstdio>

// Project
#include "dri/driver.h"
#include "map.h"
#include "motion.h"
#include "run.h"
#include "sensor.h"

// バックグラウンドタスク
[[noreturn]] static void proTask(void *);
// フォアグラウンドタスク
[[noreturn]] static void appTask(void *);

// ドライバ
Driver *driver = nullptr;
// センサ値計算クラス
Sensor *sensor = nullptr;
// 目標値生成・モーター出力クラス
Motion *motion = nullptr;
// 走行
Run *run = nullptr;

// モード選択
static uint8_t selectMode() {
  auto &sensed = sensor->getSensed();
  uint8_t mode = 0;

  auto xLastWakeTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));

    // 車輪が一定速度以上か一定速度以下で回されたらモード変更
    if (std::abs(sensed.velocity) > MODE_THRESHOLD_SPEED) {
      // モード変更
      mode = (mode + (std::signbit(sensed.velocity) ? -1 : 1)) & 0x0F;
      driver->buzzer->tone(C5, 50);
    }
    // インジケータ更新
    for (auto i = 0; i < driver->indicator->counts(); i++) {
      driver->indicator->set(i, 0, 0, (mode & (0x01 << i)) == 0 ? 0x00 : 0x0F);
    }
    driver->indicator->update();

    // 右壁センサが一定以上ならモード仮確定
    if (sensed.wall_right90.raw > MODE_THRESHOLD_WALL[PARAMETER_WALL_RIGHT90] &&
        sensed.wall_right45.raw > MODE_THRESHOLD_WALL[PARAMETER_WALL_RIGHT45]) {
      bool blink = false;
      xLastWakeTime = xTaskGetTickCount();
      while (true) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
        // 左壁センサが一定以上ならモード確定
        if (sensed.wall_left90.raw > MODE_THRESHOLD_WALL[PARAMETER_WALL_LEFT90] &&
            sensed.wall_left45.raw > MODE_THRESHOLD_WALL[PARAMETER_WALL_LEFT45]) {
          driver->buzzer->tone(C5, 100);
          return mode;
        }

        // インジケータ点滅
        for (auto i = 0; i < driver->indicator->counts(); i++) {
          uint8_t color = (mode & (0x01 << i)) == 0 ? 0x00 : 0x0F;
          driver->indicator->set(i, 0, 0, blink ? color : 0x00);
        }
        driver->indicator->update();
        blink = !blink;
      }
    }
  }
}

// パラメータ表示
// とりあえずゴール座標のみ
void printParam() {
  // 出力モードを示す
  driver->indicator->clear();
  driver->indicator->set(0, 0x0F, 0, 0);
  driver->indicator->update();

  // 表示
  printf("\x1b[2J\x1b[0;0H");
  printf(" ----- Maze Info ----- \n");
  printf("Maze Size: %d, %d\n", MAZE_SIZE_X, MAZE_SIZE_Y);
  for (auto i = 0; i < MAZE_GOAL_SIZE; i++) {
    printf("Maze Goal[%d]: %d, %d\n", i, MAZE_GOAL_X[i], MAZE_GOAL_Y[i]);
  }
}

// センサ値表示
[[noreturn]] void printSensor(bool is_csv) {
  auto &sensed = sensor->getSensed();
  auto &target = motion->getTarget();

  // 出力モードを示す
  driver->indicator->clear();
  driver->indicator->set(0, 0x0F, 0, 0);
  driver->indicator->update();

  // 画面クリア
  printf("\x1b[2J");

  // ヘッダーを出力
  if (is_csv) {
    printf("vbatt, vbatt_avg, velo, len, ang_velo, ang, x, y, r90, r45, l45, l90, tvelo, tang_velo\n");
  }

  auto xLastWakeTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));

    // 表示
    if (is_csv) {
      printf("%d, ", sensed.battery_voltage);
      printf("%d, ", sensed.battery_voltage_average);
      printf("%f, ", static_cast<double>(sensed.velocity));
      printf("%f, ", static_cast<double>(sensed.length));
      printf("%f, ", static_cast<double>(sensed.angular_velocity));
      printf("%f, ", static_cast<double>(sensed.angle));
      printf("%f, ", static_cast<double>(sensed.x));
      printf("%f, ", static_cast<double>(sensed.y));
      printf("%d, ", sensed.wall_right90.raw);
      printf("%d, ", sensed.wall_right45.raw);
      printf("%d, ", sensed.wall_left45.raw);
      printf("%d, ", sensed.wall_left90.raw);
      printf("%f, ", static_cast<double>(target.velocity));
      printf("%f, ", static_cast<double>(target.angular_velocity));
      printf("\n");
    } else {
      printf("\x1b[0;0H");
      printf(" ----- Sensor Info ----- \n");
      printf("bat_vol    : %-30d\n", sensed.battery_voltage);
      printf("bat_vol_avg: %-30d\n", sensed.battery_voltage_average);
      printf("velo    : %-30f\n", static_cast<double>(sensed.velocity));
      printf("length  : %-30f\n", static_cast<double>(sensed.length));
      printf("ang_velo: %-30f\n", static_cast<double>(sensed.angular_velocity));
      printf("angle   : %-30f\n", static_cast<double>(sensed.angle));
      printf("x: %-30f\n", static_cast<double>(sensed.x));
      printf("y: %-30f\n", static_cast<double>(sensed.y));

      printf("right90: %-30d\n", sensed.wall_right90.raw);
      printf("right45: %-30d\n", sensed.wall_right45.raw);
      printf("left45 : %-30d\n", sensed.wall_left45.raw);
      printf("left90 : %-30d\n", sensed.wall_left90.raw);

      printf("----- Target ----- \n");
      printf("velo     : %-30f\n", static_cast<double>(target.velocity));
      printf("ang velo : %-30f\n", static_cast<double>(target.angular_velocity));
    }
  }
}

// テスト直線
void testStraight() {
  driver->indicator->clear();
  driver->indicator->set(0, 0x0F, 0x0F, 0);
  driver->indicator->update();

  run->straight(MotionDirection::Forward, 180, ACCELERATION_DEFAULT, VELOCITY_DEFAULT, 0.0f);
  run->wait();
  run->stop();
}

// テストターン
void testRightTurn() {
  driver->indicator->clear();
  driver->indicator->set(0, 0x0F, 0x0F, 0);
  driver->indicator->update();

  run->turn(90, ANGULAR_ACCELERATION_DEFAULT, ANGULAR_VELOCITY_DEFAULT, MotionDirection::Right);
  run->wait();
  run->stop();
}
void testLeftTurn() {
  driver->indicator->clear();
  driver->indicator->set(0, 0x0F, 0x0F, 0);
  driver->indicator->update();

  run->turn(90, ANGULAR_ACCELERATION_DEFAULT, ANGULAR_VELOCITY_DEFAULT, MotionDirection::Left);
  run->wait();
  run->stop();
}

// テスト宴会芸
[[noreturn]] void testEnkaigei() {
  // 動作テストモードを示す
  driver->indicator->clear();
  driver->indicator->set(0, 0x0F, 0x0F, 0);
  driver->indicator->update();

  // 走行パラメータ
  MotionParameter param{};
  param.pattern = MotionPattern::Stop;
  param.max_velocity = 0;
  param.acceleration = 0;
  param.max_angular_velocity = 0;
  param.angular_acceleration = 0;
  motion->getParameterQueue().overwrite(&param);

  printSensor(true);
}

/**
 * フォアグラウンドタスク
 */
[[noreturn]] void appTask(void *) {
  driver->init_app();
  driver->indicator->enable();
  driver->buzzer->enable();
  driver->buzzer->tone(C5, 100);

  printf("mm-bluelight is started!\n");

  auto xLastWakeTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    switch (selectMode()) {
      default:
      case 0x00:
        printParam();
        break;

      case 0x01:
        printSensor(false);
        break;

      case 0x02:
        testStraight();
        break;

      case 0x03:
        testRightTurn();
        break;

      case 0x04:
        testLeftTurn();
        break;

      case 0x05:
        testEnkaigei();
        break;

      case 0x06:
      case 0x07:
      case 0x08:
      case 0x09:
      case 0x0A:
      case 0x0B:
      case 0x0C:
      case 0x0D:
      case 0x0E:
      case 0x0F:
        break;
    }
  }
}

/**
 * バックグラウンドタスク
 */
[[noreturn]] void proTask(void *) {
  driver->init_pro();
  vTaskDelay(pdMS_TO_TICKS(500));

  // モーター有効化
  driver->motor_right->enable();
  driver->motor_left->enable();

  // センサ初期化
  sensor->setup();

  // TODO: 止める手段を作る (フラッシュ読み書きのため)
  auto xLastWakeTime = xTaskGetTickCount();
  while (true) {
    sensor->update();
    motion->update();
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
  }
}

/**
 * エントリーポイント
 */
extern "C" void app_main(void) {
  driver = new Driver();
  sensor = new Sensor(driver);
  motion = new Motion(driver, sensor);
  run = new Run(sensor, motion);
  xTaskCreatePinnedToCore(proTask, "proTask", 8192, nullptr, 20, nullptr, 0);
  xTaskCreatePinnedToCore(appTask, "appTask", 8192, nullptr, 20, nullptr, 1);
}
