#ifndef MQTT_H
#define MQTT_H
#include <Actor.h>
#include <Erc.h>
#include <esp_mqtt.h>

#define HOST_LENGTH 40
#define USER_LENGTH 40
#define PASSWORD_LENGTH 40
#define CLIENT_ID_LENGTH 40

class Mqtt : public Actor {
  static Mqtt* _me;
  Str _host;
  uint16_t _port;
  Str _user;
  Str _password;
  Str _clientId;
  uid_t _wifi;
  bool _connected;

 public:
  Mqtt(const char* name);
  void init();
  Erc configure(const char* host, uint16_t port, const char* user,
                const char* password);
  void setWifi(uid_t wifi) { _wifi = wifi; }

  void onConnected();
  void onDisconnected();
  void onMessageReceived(const char* topic, uint8_t* payload, uint32_t size);

  static void onStatusChange(esp_mqtt_status_t status);
  static void onMessage(const char* topic, uint8_t* payload, uint32_t size);

  void publish(const char* topic, uint8_t* payload, size_t len, int qos,
               bool retained);
  void setup();
  void onEvent(Cbor& cbor);
};
#endif