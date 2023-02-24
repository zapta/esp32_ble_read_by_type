
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <stdio.h>

static constexpr auto TAG = "app_main";

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

extern "C" void app_main() {
  gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_NUM_2, 1);
  nvs_init();
  host::setup();
  for (uint32_t i = 0;; i++) {
    vTaskDelay(10);
    const uint32_t blink_mask = host::is_connected() ? 0x0001 : 0x0008;
    gpio_set_level(GPIO_NUM_2, i & blink_mask ? 1 : 0);
  }
}
