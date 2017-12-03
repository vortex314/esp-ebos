/*
 * Sys.cpp
 *
 *  Created on: May 15, 2016
 *      Author: lieven
 */

#include <Log.h>
#include <Sys.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <esp_timer.h>

#include <soc/efuse_reg.h>
#include <string.h>
/*
 *
 *  ATTENTION : LOGF call Sys::hostname, could invoke another call to
 * Sys::hostname with LOGF,....
 *
 */
char Sys::_hostname[30];
uint64_t Sys::_boot_time = 0;

uint64_t Sys::micros() { return esp_timer_get_time(); }

uint64_t Sys::millis() { return micros() / 1000; }

uint64_t Sys::now() { return _boot_time + Sys::millis(); }

void Sys::setNow(uint64_t n) { _boot_time = n - Sys::millis(); }

void Sys::hostname(const char *h) { strncpy(_hostname, h, strlen(h) + 1); }

void Sys::setHostname(const char *h) { strncpy(_hostname, h, strlen(h) + 1); }

uint32_t Sys::getFreeHeap() { return heap_caps_get_free_size(MALLOC_CAP_8BIT); }

uint32_t Sys::getSerialId() {
  union {
    uint64_t _chipmacid;
    uint32_t _mac[2];
  };

  esp_efuse_mac_get_default((uint8_t *)(&_chipmacid));
  return _mac[1];
}

// void Sys::reset() { while(true);};

void Sys::init() { sprintf(_hostname, "ESP%X", Sys::getSerialId()); }

const char *Sys::hostname() { return _hostname; }

void Sys::delay(unsigned int delta) {
  uint32_t end = Sys::millis() + delta;
  while (Sys::millis() < end)
    ;
}

extern "C" uint64_t SysMillis() { return Sys::millis(); }
