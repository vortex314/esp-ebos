#include <Wifi.h>

#include <stdlib.h>
#include <string.h>

#include <esp_event_loop.h>
//#include <esp_mqtt.h>
#include <esp_wifi.h>

Wifi *Wifi::_me = 0;

Wifi::Wifi(const char *name) : Actor(name), _ssid(32), _pswd(64) { _me = this; }

Erc Wifi::configure(const char *ssid, const char *pswd) {
  _ssid = ssid;
  _pswd = pswd;
  return E_OK;
}

esp_err_t Wifi::event_handler(void *ctx, system_event_t *event) {
  switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
      esp_wifi_connect();
      break;

    case SYSTEM_EVENT_STA_GOT_IP:
      eb.publish(_me->id(), H("connected"));

      break;

    case SYSTEM_EVENT_STA_DISCONNECTED:
      eb.publish(_me->id(), H("disconnected"));

      esp_wifi_connect();
      break;

    default:
      break;
  }

  return ESP_OK;
}

void Wifi::init() {}

void Wifi::setup() {
  tcpip_adapter_init();
  ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

  wifi_config_t wifi_config;
  strncpy((char *)wifi_config.sta.ssid, _ssid.c_str(), 32);
  strncpy((char *)wifi_config.sta.password, _pswd.c_str(), 64);

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());
}

void Wifi::onEvent(Cbor &ev) {}