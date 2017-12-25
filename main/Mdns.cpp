#include <Config.h>
#include <Mdns.h>

#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "mdns.h"
#include "nvs_flash.h"

mdns_server_t* mdns_server = NULL;

void start_mdns_service() {
  // initialize mDNS service on STA interface
  esp_err_t err = mdns_init(TCPIP_ADAPTER_IF_STA, &mdns_server);
  if (err) {
    ERROR("MDNS Init failed: %d\n", err);
    return;
  }

  // set hostname
  mdns_set_hostname(mdns_server, Sys::hostname());
  // set default instance
  mdns_set_instance(mdns_server, "Limero-xxx");
}

Mdns::Mdns(const char* name) : Actor(name) {}

void Mdns::setup() {
  config.setNameSpace("mdns");

  eb.onDst(id()).call(this);
  eb.onSrc(_wifi).call(this);
  timeout(UINT32_MAX);
}

void Mdns::setWifi(uid_t wifi) { _wifi = wifi; }

void Mdns::onEvent(Cbor& msg) {
  if (eb.isEvent(_wifi, H("connected"))) {
    start_mdns_service();
  }
}

void Mdns::init() {}