#include <cstdio>

#include "dri/driver.h"
#include "map.h"
#include "motion.h"
#include "sensor.h"

Driver *driver = nullptr;
Sensor *sensor = nullptr;
Motion *motion = nullptr;

[[noreturn]] void proTask(void *) {
  driver->init_pro();
  vTaskDelay(pdMS_TO_TICKS(500));

  // センサ初期化
  sensor->setup();

  // モーター有効
  driver->motor_left->enable();
  driver->motor_right->enable();

  auto xLastWakeTime = xTaskGetTickCount();
  while (true) {
    sensor->update();
    motion->update();
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
  }
}

[[noreturn]] void appTask(void *) {
  driver->init_app();
  vTaskDelay(pdMS_TO_TICKS(1000));

  driver->indicator->enable();
  driver->buzzer->enable();

  // センサ値
  Sensed sensed{};
  // モード
  uint8_t mode = 0;

  auto xLastWakeTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    switch (mode) {
      case 0x00:
        break;
      case 0x01:
        break;
      case 0x02:
        break;
      case 0x03:
        break;
      case 0x04:
        break;
      case 0x05:
        break;
      case 0x06:
        break;
      case 0x07:
        break;
      case 0x08:
        break;
      case 0x09:
        break;
      case 0x0A:
        break;
      case 0x0B:
        break;
      case 0x0C:
        break;
      case 0x0D:
        break;
      case 0x0E:
        break;
      case 0x0F:
        break;
    }
    sensor->getSensedQueue().peek(&sensed, pdMS_TO_TICKS(1));
    if (sensed.velocity > 0.1f) {
      mode = (mode + 1) & 0x0F;
    } else if (sensed.velocity < -0.1f) {
      mode = (mode - 1) & 0x0F;
    } else {
    }
    for (auto i = 0; i < driver->indicator->counts(); i++) {
      driver->indicator->set(i, 0, 0, (mode & (0x01 << i)) == 0 ? 0x00 : 0x0F);
    }
    driver->indicator->update();
  }
}

extern "C" void app_main(void) {
  driver = new Driver();
  sensor = new Sensor(driver);
  motion = new Motion(driver, sensor->getSensedQueue());
  xTaskCreatePinnedToCore(proTask, "proTask", 8192, nullptr, 20, nullptr, 0);
  xTaskCreatePinnedToCore(appTask, "appTask", 8192, nullptr, 20, nullptr, 1);
}
