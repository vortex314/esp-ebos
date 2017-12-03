// file: main.cpp
#include <LedBlinker.h>
#include <Sys.h>
#include <System.h>
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_int_wdt.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "nvs_flash.h"

#include <MqttJson.h>
#include <cJSON.h>

Uid uid(200);
EventBus eb(32000, 1024);
Log logger(256);

LedBlinker led;
System sys("system");

//================================================================================
#include <Hardware.h>

#include "HMC5883L.h"

class Compass : public Actor {
  HMC5883L _hmc;

 public:
  Compass(const char* name, I2C& i2c) : Actor(name), _hmc(i2c) {
    uid.add("x");
    uid.add("y");
    uid.add("z");
  };
  virtual ~Compass(){};

  void setup() {
    timeout(5000);
    if (_hmc.init()) {
      INFO("HMC5883L initialized.")
    } else {
      ERROR("HMC5883L initialization failed.");
    }
    _hmc.setRange(HMC5883L_RANGE_1_3GA);

    // Set measurement mode
    _hmc.setMeasurementMode(HMC5883L_CONTINOUS);

    // Set data rate
    _hmc.setDataRate(HMC5883L_DATARATE_30HZ);

    // Set number of samples averaged
    _hmc.setSamples(HMC5883L_SAMPLES_8);

    // Set calibration offset. See HMC5883L_calibration.ino
    _hmc.setOffset(0, 0);
  }
  void onEvent(Cbor& event) {
    struct Vector v;
    timeout(100);
    v = _hmc.readNormalize();
    INFO("%f:%f:%f", v.XAxis, v.YAxis, v.ZAxis);
    eb.publicEvent(id(), H("x")).addKeyValue(H("x"), v.XAxis);
    eb.send();
    eb.publicEvent(id(), H("y")).addKeyValue(H("y"), v.YAxis);
    eb.send();
    eb.publicEvent(id(), H("z")).addKeyValue(H("z"), v.ZAxis);
    eb.send();
    //   _hmc.printReg();
  }
};

//================================================================================
#include <Hardware.h>

#include "HCSR04.h"

class UltraSonic : public Actor {
  HCSR04 _hcsr;

 public:
  UltraSonic(const char* name, DigitalOut& pinTrigger, DigitalIn& pinEcho)
      : Actor(name), _hcsr(pinTrigger, pinEcho) {
    uid.add("cm");
    uid.add("distance");
  };
  virtual ~UltraSonic(){};

  void setup() {
    timeout(5000);
    _hcsr.init();
  }
  void onEvent(Cbor& event) {
    timeout(100);
    INFO("distance : %lld µsec = %f cm ", _hcsr.getTime(),
         _hcsr.getCentimeters());
    eb.publicEvent(id(), H("distance"))
        .addKeyValue(H("distance"), _hcsr.getCentimeters());
    eb.send();
    _hcsr.trigger();
  }
};

//===============================================================================
#include <Mqtt.h>
#include <Wifi.h>

Wifi wifi("Wifi");
Mqtt mqtt("mqtt");

I2C i2c(I2C_NUM_0, 25, 26);
// UEXT 1 port 0 scl:15  sda:4
// UEXT 2 port 0 scl:25  sda:26
// UEXT 3 port 0 scl:22  sda:5
Compass compass("compass", i2c);

DigitalIn echoPin(4);
DigitalOut triggerPin(15);
UltraSonic us("US", triggerPin, echoPin);  // SCL -> trigger,SDA -> echo
MqttJson gateway("gateway", 1024);

extern "C" void setup() {
  // loadHardware();

  Sys::init();
  eb.setup();

  wifi.configure("Merckx3", "LievenMarletteEwoutRonald");
  wifi.setup();

  mqtt.setWifi(wifi.id());
  mqtt.configure("limero.ddns.net", 1883, "", "");
  mqtt.setup();

  gateway.setMqttId(mqtt.id());
  gateway.setup();

  led.setMqtt(mqtt.id());
  led.setWifi(wifi.id());

  compass.setup();
  us.setup();

  logger.setLogLevel('D');
  eb.onAny().call([](Cbor& msg) {  // Log all events
    Str str(256);
    eb.log(str, msg);
    DEBUG("%s", str.c_str());
  });

  sys.setup();

  led.setup();

  while (1) {
    eb.eventLoop();
    esp_task_wdt_reset();
    vTaskDelay(1);
  }
}

void measure(const char* s) {
  static uint64_t startTime;
  uint32_t delta;
  delta = Sys::millis() - startTime;
  startTime = Sys::millis();
  if (delta > 20) WARN(" slow %s :  %d msec", s, delta);
}
