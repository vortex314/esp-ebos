/*
 * Config.cpp
 *
 *  Created on: Jul 12, 2016
 *      Author: lieven
 */

#include "Config.h"
#include <EventBus.h>

#include <Sys.h>

#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"
#define EEPROM_SIZE 512
#define EEPROM_MAGIC 0xDEADBEEF
#define KEY_SIZE 40
#define VALUE_SIZE 60

Config::Config() : _jsonBuffer(1024), _nameSpace(30), _loaded(false) {}

Config::~Config() {}

void Config::initMagic() {}

bool Config::checkMagic() { return false; }

void Config::clear() { load(); }

void Config::load() {
  if (_loaded) {
    char _buffer[1024];
    _root->prettyPrintTo(_buffer, sizeof(_buffer));
    //    DEBUG(" config object : %s", _buffer);
    return;
  }

  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
    // NVS partition was truncated and needs to be erased
    // Retry nvs_flash_init
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  nvs_handle my_handle;
  err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK) {
    ERROR("Error (%d) opening NVS handle!\n", err);
  } else {
    INFO("NVS storage open \n");

    uint32_t required_size;
    err = nvs_get_str(my_handle, "config", _charBuffer, &required_size);

    switch (err) {
      case ESP_OK:
        INFO(" config : %s ", _charBuffer);
        break;
      case ESP_ERR_NVS_NOT_FOUND:
        WARN(" no config in NVS ")
        strcpy(_charBuffer, "{}");
        break;
      default:
        ERROR(" error read NVS : %d ", err);
        strcpy(_charBuffer, "{}");
    }
    nvs_close(my_handle);
  }
  _loaded = true;
  _root = &_jsonBuffer.parseObject(_charBuffer);
  if (!_root->success()) {
    _root = &_jsonBuffer.parseObject("{}");
  }
}

void Config::save() {
  esp_err_t err;
  char buffer[1024];
  _root->printTo(buffer, sizeof(buffer));
  nvs_handle my_handle;
  err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK) {
    ERROR("Error (%d) opening NVS handle!\n", err);
    return;
  }
  err = nvs_set_str(my_handle, "config", buffer);
  if (err) {
    ERROR(" nvs_set_str(my_handle,  'config', buffer): %d ", err);
  }

  err = nvs_commit(my_handle);
  if (err) {
    ERROR(" nvs_commit(my_handle) : %d ", err);
    nvs_close(my_handle);
    return;
  }
  INFO(" config saved : %s ", buffer);
}

const char* Config::clone(const char* s) {
  char* p = (char*)_jsonBuffer.alloc(strlen(s) + 1);
  strcpy(p, s);
  return p;
}

//======================================NAMESPACE
JsonObject& Config::nameSpace() {
  if (_root->containsKey(_nameSpace.c_str())) {
    JsonObject& jso = (*_root)[_nameSpace.c_str()];
    return jso;
  } else {
    JsonObject& jso = _root->createNestedObject(clone(_nameSpace.c_str()));
    return jso;
  }
}

void Config::setNameSpace(const char* ns) { _nameSpace = ns; }

const char* Config::getNameSpace() { return _nameSpace.c_str(); }
//==================================================
bool Config::hasKey(const char* key) {
  JsonObject& ns = nameSpace();
  if (ns.containsKey(key)) {
    return true;
  }
  return false;
}

void Config::remove(const char* key) {
  load();
  JsonObject& ns = nameSpace();
  ns.remove(key);
  INFO(" Config => SAVE  remove %s ", key);
}

void Config::set(const char* key, const char* value) {
  JsonObject& ns = nameSpace();
  ns[clone(key)] = clone(value);
}

void Config::set(const char* key, Str& value) {
  load();
  set(key, value.c_str());
}

void Config::set(const char* key, uint32_t value) {
  load();
  JsonObject& ns = nameSpace();
  ns[clone(key)] = value;
}

void Config::set(const char* key, int32_t value) {
  load();
  JsonObject& ns = nameSpace();
  ns[clone(key)] = value;
}

void Config::set(const char* key, double value) {
  load();
  JsonObject& ns = nameSpace();
  INFO(" SET %s=%f ", key, value);
  ns[clone(key)] = value;
}

void Config::get(const char* key, Str& value, const char* defaultValue) {
  load();
  JsonObject& ns = nameSpace();
  if (ns.containsKey(key)) {
    value = ns[key];
  } else {
    set(key, defaultValue);
    value = defaultValue;
  }
  INFO(" %s.%s = '%s' ", _nameSpace.c_str(), key, value.c_str());
}

void Config::get(const char* key, int32_t& value, int32_t defaultValue) {
  load();
  JsonObject& ns = nameSpace();
  if (ns.containsKey(key)) {
    value = ns[key];
  } else {
    set(key, defaultValue);
    value = defaultValue;
  }
  INFO(" %s.%s = %d ", _nameSpace.c_str(), key, value);
}

void Config::get(const char* key, uint32_t& value, uint32_t defaultValue) {
  load();
  JsonObject& ns = nameSpace();
  if (ns.containsKey(key)) {
    value = ns[key];
  } else {
    set(key, defaultValue);
    value = defaultValue;
  }
  INFO(" %s.%s = %u ", _nameSpace.c_str(), key, value);
}

void Config::get(const char* key, double& value, double defaultValue) {
  load();
  JsonObject& ns = nameSpace();
  if (ns.containsKey(key)) {
    value = ns[key];
  } else {
    set(key, defaultValue);
    value = defaultValue;
  }
  INFO(" %s.%s = %f ", _nameSpace.c_str(), key, value);
}

void Config::print(Str& str) { load(); }

void Config::printPretty(Str& str) { load(); }

Config config;
