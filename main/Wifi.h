#ifndef WIFI_H
#define WIFI_H

#include <stdio.h>
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"

#include <EventBus.h>
#include <Str.h>

class Wifi : public Actor {
  Str _ssid;
  Str _pswd;
  static Wifi* _me;

 public:
  Wifi(const char* name);
  void init();
  void setup();
  void onEvent(Cbor& cbor);
  Erc configure(const char* ssid, const char* pswd);
  static esp_err_t event_handler(void*, system_event_t*);
};

#endif