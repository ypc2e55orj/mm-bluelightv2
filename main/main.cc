#include <cstdio>

#include "dri/driver.h"
#include "map.h"
#include "search.h"

[[noreturn]] void mainTask(void *) {
  Driver.init_app();
  auto xLastWakeTime = xTaskGetTickCount();
  while (true) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
  }
}

extern "C" void app_main(void) {
  Driver.init_pro();
  xTaskCreatePinnedToCore(mainTask, "mainTask", 8192, nullptr, 20, nullptr, 1);
}
