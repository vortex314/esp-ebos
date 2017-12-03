/*
 * LedBlinker.cpp
 *
 *  Created on: Jul 1, 2016
 *      Author: lieven
 */

#include "LedBlinker.h"
#include "Property.h"
#include "driver/gpio.h"

#ifdef ESP8266
#define PIN 16
#else
#define PIN 2
#endif

static const char* labels[] = {"interval"};

LedBlinker::LedBlinker() : Actor("Led") {
  _interval = 100;
  _isOn = true;
}

LedBlinker::~LedBlinker() {}

void LedBlinker::setup() {
  init();
  timeout(100);
  eb.onSrc(_wifi).call(this);
  eb.onSrc(_mqtt).call(this);

  uid.add(labels, sizeof(labels) / sizeof(const char*));
  Property<uint32_t>::build(_interval, id(), H("interval"), 20000);
}

void LedBlinker::init() {
  gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
}

void LedBlinker::onEvent(Cbor& cbor) {
  if (timeout()) {
    if (_isOn) {
      _isOn = false;
      gpio_set_level(GPIO_NUM_2, 1);
    } else {
      _isOn = true;
      gpio_set_level(GPIO_NUM_2, 0);
    }
    timeout(_interval);
  } else if (eb.isEvent(_wifi, H("connected"))) {
    setInterval(100);
  } else if (eb.isEvent(_wifi, H("disconnected"))) {
    setInterval(50);
  } else if (eb.isEvent(_mqtt, H("connected"))) {
    setInterval(1000);
  } else if (eb.isEvent(_mqtt, H("disconnected"))) {
    setInterval(100);
  }
}

void LedBlinker::setInterval(uint32_t interval) { _interval = interval; }

void LedBlinker::setWifi(uid_t wifi) { _wifi = wifi; }

void LedBlinker::setMqtt(uid_t mqtt) { _mqtt = mqtt; }
