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
  auto xLastWakeTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
  }
}

extern "C" void app_main(void) {
  driver = new Driver();
  sensor = new Sensor(driver);
  motion = new Motion(driver, sensor->getSensedQueue());
  xTaskCreatePinnedToCore(proTask, "proTask", 8192, nullptr, 20, nullptr, 0);
  xTaskCreatePinnedToCore(appTask, "appTask", 8192, nullptr, 20, nullptr, 1);
}
