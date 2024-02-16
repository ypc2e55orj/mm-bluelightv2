// C++
#include <array>
#include <cstdio>

// Project
#include "dri/driver.h"
#include "map.h"
#include "motion.h"
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
  MotionTarget target{};

  // 出力モードを示す
  driver->indicator->clear();
  driver->indicator->set(0, 0x0F, 0, 0);
  driver->indicator->update();

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
      printf("%d, ", sensed.wall_left90.raw);
      printf("%d, ", sensed.wall_left45.raw);
      printf("%d, ", sensed.wall_right45.raw);
      printf("%d, ", sensed.wall_right90.raw);
      printf("%f, ", static_cast<double>(target.velocity));
      printf("%f\n", static_cast<double>(target.angular_velocity));
    } else {
      printf("\x1b[2J\x1b[0;0H");
      printf(" ----- Sensor Info ----- \n");
      printf("bat_vol    : %d\n", sensed.battery_voltage);
      printf("bat_vol_avg: %d\n", sensed.battery_voltage_average);
      printf("velo    : %f\n", static_cast<double>(sensed.velocity));
      printf("length  : %f\n", static_cast<double>(sensed.length));
      printf("ang_velo: %f\n", static_cast<double>(sensed.angular_velocity));
      printf("angle   : %f\n", static_cast<double>(sensed.angle));
      printf("x: %f\n", static_cast<double>(sensed.x));
      printf("y: %f\n", static_cast<double>(sensed.y));

      printf("left90 : %d\n", sensed.wall_left90.raw);
      printf("left45 : %d\n", sensed.wall_left45.raw);
      printf("right45: %d\n", sensed.wall_right45.raw);
      printf("right90: %d\n", sensed.wall_right90.raw);

      printf("----- Target ----- \n");
      printf("velo     : %f\n", static_cast<double>(target.velocity));
      printf("ang velo : %f\n", static_cast<double>(target.angular_velocity));
    }
  }
}

// テスト直線
[[noreturn]] void testStraight() {
  // 動作テストモードを示す
  driver->indicator->clear();
  driver->indicator->set(0, 0x0F, 0x0F, 0);
  driver->indicator->update();

  // モーター有効
  driver->motor_left->enable();
  driver->motor_right->enable();

  // 走行パラメータ
  MotionParameter param{};
  param.pattern = MotionPattern::Straight;
  param.max_velocity = VELOCITY_DEFAULT;
  param.acceleration = ACCELERATION_DEFAULT;
  param.max_angular_velocity = ANGULAR_VELOCITY_DEFAULT;
  param.angular_acceleration = ANGULAR_ACCELERATION_DEFAULT;
  motion->getParameterQueue().overwrite(&param);

  printSensor(true);
}

// テスト宴会芸
[[noreturn]] void testEnkaigei() {
  // 動作テストモードを示す
  driver->indicator->clear();
  driver->indicator->set(0, 0x0F, 0x0F, 0);
  driver->indicator->update();

  // モーター有効
  driver->motor_left->enable();
  driver->motor_right->enable();

  // 走行パラメータ
  MotionParameter param{};
  param.pattern = MotionPattern::Turn;
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
  vTaskDelay(pdMS_TO_TICKS(1000));

  driver->indicator->enable();
  driver->buzzer->enable();

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

      case 0x02:
        testStraight();

      case 0x03:
        testEnkaigei();

      case 0x04:
      case 0x05:
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
  xTaskCreatePinnedToCore(proTask, "proTask", 8192, nullptr, 20, nullptr, 0);
  xTaskCreatePinnedToCore(appTask, "appTask", 8192, nullptr, 20, nullptr, 1);
}
