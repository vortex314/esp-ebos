#include <stdio.h>
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"

// esp_err_t event_handler(void *ctx, system_event_t *event) { return ESP_OK; }

extern void setup();

#include "driver/spi_master.h"
#include "esp_system.h"

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK 18
#define PIN_NUM_CS 21

void app_main() {
  nvs_flash_init();
  setup();
}
