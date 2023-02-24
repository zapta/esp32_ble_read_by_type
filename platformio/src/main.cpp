
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <stdio.h>

static constexpr auto TAG = "main";

namespace ble_host {
void setup();
}

void nvs_init() {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "nvs_flash_init() ok.");
    return;
  }

  // This is the initial creation in new devices.
  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
      err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGW(TAG, "nvs_flash_init() err 0x%x %s. Erasing nvs...", err,
        esp_err_to_name(err));
    // This should not fail.
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_LOGI(TAG, "nvs erased. Initializing again...");
    // This should not fail.
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_LOGI(TAG, "nvs_flash_init() ok.");
    return;
  }
}

// static void setup() {
//   nvs_init();
//   ble_host::setup();
// }

// static void loop() { vTaskDelay(100); }

extern "C" void app_main() {
  nvs_init();
  ble_host::setup();

  for (int i = 0; ; i++) {
    ESP_LOGI(TAG, "%03d. loop", i);
    vTaskDelay(100);
    // loop();
  }
}
