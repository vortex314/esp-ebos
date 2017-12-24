

#include <Config.h>
#include <EventBus.h>
#include <Mqtt.h>

Mqtt *Mqtt::_me = 0;

Mqtt::Mqtt(const char *name)
    : Actor(name),
      _host(HOST_LENGTH),
      _port(1883),
      _user(USER_LENGTH),
      _password(PASSWORD_LENGTH),
      _clientId(CLIENT_ID_LENGTH),
      _willTopic(30),
      _willMessage(30),
      _willQos(0),
      _willRetain(false),
      _keepAlive(20) {
  _me = this;
  _connected = false;
}

void Mqtt::init(){};

Erc Mqtt::configure(const char *host, uint16_t port, const char *user,
                    const char *password) {
  _host = host;
  _port = port;
  _user = user;
  _password = password;
  return E_OK;
}

void Mqtt::onStatusChange(esp_mqtt_status_t status) {
  switch (status) {
    case ESP_MQTT_STATUS_CONNECTED:
      _me->onConnected();
      break;
    case ESP_MQTT_STATUS_DISCONNECTED:
      _me->onDisconnected();
      break;
  }
}

void Mqtt::onMessage(const char *topic, uint8_t *payload, uint32_t size) {
  _me->onMessageReceived(topic, payload, size);
  INFO(" mqtt topic received : %s ", topic);
}

void Mqtt::onConnected() {
  _connected = true;
  eb.publish(id(), H("connected"));
}

void Mqtt::onDisconnected() {  //
  _connected = false;
  eb.publish(id(), H("disconnected"));
};

void Mqtt::onMessageReceived(const char *topic, uint8_t *payload,
                             uint32_t size) {
  Bytes data(0);
  data.map(payload, size);
  eb.event(id(), H("published"))
      .addKeyValue(H("topic"), topic)
      .addKeyValue(H("message"), data);
  eb.send();
}

void Mqtt::setup() {
  config.setNameSpace("mqtt");
  config.get("host", _host, "limero.ddns.net");
  config.get("port", _port, 1883);
  _clientId = Sys::hostname();
  config.get("user", _user, "");
  config.get("password", _password, "");
  _willTopic = "src/";
  _willTopic += Sys::hostname();
  _willTopic += "/system/alive";
  //    config.get("mqtt.willTopic",_willTopic,_willTopic.c_str());
  config.get("willMessage", _willMessage, "false");
  config.get("keepAlive", _keepAlive, 20);
  esp_mqtt_init(onStatusChange, onMessage, 256, 2000);
  eb.onDst(id()).call(this);
  eb.onSrc(_wifi).call(this);
  timeout(UINT32_MAX);
}

void Mqtt::publish(const char *topic, uint8_t *payload, size_t len, int qos,
                   bool retained) {
  if (!esp_mqtt_publish(topic, payload, len, qos, retained)) {
    ERROR(" mqtt publish failed : %s size:%d ", topic, len);
    esp_mqtt_stop();
    esp_mqtt_start(_host.c_str(), _port, _clientId.c_str(), _user.c_str(),
                   _password.c_str());
  } else {
    // INFO(" published %s", topic);
  }
}

void Mqtt::onEvent(Cbor &ev) {
  if (eb.isEvent(_wifi, H("connected"))) {
    esp_mqtt_start(_host.c_str(), _port, _clientId.c_str(), _user.c_str(),
                   _password.c_str());
  } else if (eb.isEvent(_wifi, H("disconnected"))) {
    esp_mqtt_stop();
    onDisconnected();
  } else if (eb.isRequest(id(), H("publish"))) {
    Str topic(40);
    Bytes message(200);
    if (ev.getKeyValue(H("topic"), topic) &&
        ev.getKeyValue(H("message"), message) && _connected) {
      publish(topic.c_str(), message.data(), message.length(), 1, false);
      eb.reply().addKeyValue(EB_ERROR, E_OK);
      eb.send();
    } else {
      WARN(" no topic or message found or disconnected ");
    }
  } else if (eb.isRequest(id(), H("subscribe"))) {
    Str topic(80);
    if (ev.getKeyValue(H("topic"), topic)) {
      if (!esp_mqtt_subscribe(topic.c_str(), 0) && _connected) {
        ERROR(" mqtt subscribe failed");
      } else {
        eb.reply().addKeyValue(EB_ERROR, E_OK);
        eb.send();
      }
    } else {
      Str str(256);
      eb.log(str, ev);
      DEBUG("%s", str.c_str());
      WARN(" no topic  found or disconnected ");
    }
  } else {
    Str str(256);
    eb.log(str, ev);
    DEBUG("%s", str.c_str());
    INFO(" not for me ");
  }
}
