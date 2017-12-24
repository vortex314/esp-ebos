// file: main.cpp
#include <LedBlinker.h>
#include <Sys.h>
#include <System.h>
#include <esp_attr.h>
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
#include <Property.h>
//#include <cJSON.h>
#include <Config.h>
#include <Mdns.h>

Uid uid(200);
EventBus eb(32000, 1024);
Log logger(256);

LedBlinker led;
System sys("system");

void logCbor(const char* prefix, Cbor& cbor) {
  uid_t key, value;
  Str str(100);
  Cbor::PackType ct;

  cbor.offset(0);

  while (cbor.hasData()) {
    cbor.get(key);

    const char* label = uid.label(key);
    str.append(" | ").append(label).append(":");
    if (label[0] == '%') {
      cbor.get(value);
      str.append("").append(uid.label(value));
    } else {
      ct = cbor.tokenToString(str);
      if (ct == Cbor::P_BREAK || ct == Cbor::P_ERROR) break;
    }
    str.append("");
    if (cbor.hasData()) str << "";
  };
  INFO("%s ::: %s", prefix, str.c_str());
}

//================================================================================
#include <Hardware.h>

#include "HCSR04.h"

class Drive : public Actor {
  DigitalOut& _left;
  DigitalOut& _right;
  DigitalOut& _enable;
  ADC& _current;
  ADC& _position;
  uint32_t _count;
  float _min, _max;
  float _target;
  float _direction;
  float _voltage;

 public:
  Drive(const char* name, DigitalOut& left, DigitalOut& right,
        DigitalOut& enable, ADC& current, ADC& position)
      : Actor(name),
        _left(left),
        _right(right),
        _enable(enable),
        _current(current),
        _position(position) {
    uid.add("cm");
    uid.add("distance");
    _count = 0;
    _target = 1.8;
    _min = 1.1;
    _max = 2.35;
    _direction = 0;
  };
  virtual ~Drive(){};

  void init() {
    _left.init();
    _right.init();
    _enable.init();
    _position.init();
    _current.init();
    _left.write(0);
    _right.write(0);
    _enable.write(0);
    _target = 1.9;
    _direction = 0.0;
    calcTarget(0);
  }
  void setup() {
    timeout(5000);
    init();
    uid.add("voltage");
    uid.add("target");
    uid.add("direction");
    eb.onDst(id()).call(this);
    Property<float>::build(_target, id(), H("target"), 100);
    Property<float>::build(_direction, id(), H("direction"), 100);
    Property<float>::build(_voltage, id(), H("voltage"), 100);
  }

  void calcTarget(float v) {
    if (v < -100.0) {
      v = -100.0;
    } else if (v > 100) {
      v = 100.0;
    };
    float v1 = (v + 100) / 200;  // 0->1.0
    _target = (v1 * (_max - _min)) + _min;
  }

  float absolute(float f) {
    if (f > 0) return f;
    return -f;
  }
  void onEvent(Cbor& msg) {
    _voltage = _position.getValue();
    if (eb.isRequest(id(), H("set"))) {
      logCbor("RCV", msg);
      float newDirection;

      if (msg.getKeyValue(H("direction"), newDirection)) {
        calcTarget(newDirection);
        eb.reply().addKeyValue(EB_ERROR, E_OK);
        eb.send();
      } else {
        ERROR(" set failed, no target found ");
      }
    } else {
      timeout(10);
      float delta = absolute(_voltage - _target);
      if (delta < 0.05) {
        _left.write(1);
        _right.write(1);
        _enable.write(1);
      } else if (_voltage < _target) {
        _left.write(0);
        _right.write(1);
        _enable.write(1);
      } else if (_voltage > _target) {
        _left.write(1);
        _right.write(0);
        _enable.write(1);
      }
      _count++;

      /* if ((_count % 100) == 0) {
         _direction += 10;
         if (_direction >= 150) _direction = -150;
         calcTarget(_direction);
       }*/
    }
  }
};

DigitalOut left(25);
DigitalOut right(26);
DigitalOut enable(32);
ADC current(36);
ADC position(33);
Drive drive("drive", left, right, enable, current, position);
#include <DWM1000_Anchor.h>

//===============================================================================

#include "HMC5883L.h"

class Compass : public Actor {
  HMC5883L _hmc;

 public:
  Compass(const char* name, I2C& i2c) : Actor(name), _hmc(i2c) {
    uid.add("x");
    uid.add("y");
    uid.add("z");
  };
  Compass(const char* name, Connector& connector)
      : Actor(name), _hmc(connector) {
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
    timeout(500);
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

#include "HCSR04.h"

class UltraSonic : public Actor {
  HCSR04 _hcsr;

 public:
  UltraSonic(const char* name, Connector& connector)
      : Actor(name), _hcsr(connector) {
    uid.add("cm");
    uid.add("distance");
  };
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
    timeout(500);
    float cm = _hcsr.getCentimeters();
    if (cm < 400 && cm > 0) {
      INFO("distance : %lld µsec = %f cm ", _hcsr.getTime(),
           _hcsr.getCentimeters());
      eb.publicEvent(id(), H("distance"))
          .addKeyValue(H("distance"), _hcsr.getCentimeters());
      eb.send();
      eb.request(drive.id(), H("set"), id())
          .addKeyValue(H("direction"), _hcsr.getCentimeters() - 100);
      eb.send();
    }
    _hcsr.trigger();
  }
};
//================================================================================

#include <Mqtt.h>
#include <Wifi.h>

Wifi wifi("Wifi");
Mqtt mqtt("mqtt");
Mdns mdns("mdns");

// I2C i2c(I2C_NUM_0, 15, 4);
// UEXT 1 port 0 scl:15  sda:4
// UEXT 2 port 0 scl:25  sda:26
// UEXT 3 port 0 scl:22  sda:5
Connector uext1(1);
Connector uext2(2);
Connector uext3(3);

UltraSonic us("US", uext3);  // SCL -> trigger,SDA -> echo
Compass compass("compass", uext2);

SPI& spi = SPI::create(19, 23, 18, 21);
DigitalIn irq(16);
DigitalOut reset(17);

DRAM_ATTR DWM1000_Anchor dwm1000Anchor("dwm1000", spi, irq, reset);

MqttJson gateway("gateway", 1024);

void wait() {
  while (1) {
    esp_task_wdt_reset();
    vTaskDelay(1);
  }
}

extern "C" void setup() {
  // loadHardware();

  config.load();

  Sys::init();
  eb.setup();
  logger.setLogLevel('I');
  eb.onAny().call([](Cbor& msg) {  // Log all events
    Str str(256);
    eb.log(str, msg);
    DEBUG("%s", str.c_str());
  });

  Str strHostname(30);
  char hn[20];
  sprintf(hn, "ESP%X", Sys::getSerialId());

  config.setNameSpace("sys");
  config.get("host", strHostname, hn);
  Sys::hostname(strHostname.c_str());

  //  wifi.configure("Merckx3", "LievenMarletteEwoutRonald");
  wifi.setup();

  mdns.setWifi(wifi.id());
  mdns.setup();

  mqtt.setWifi(wifi.id());
  mqtt.setup();

  gateway.setMqttId(mqtt.id());
  gateway.setup();

  led.setMqtt(mqtt.id());
  led.setWifi(wifi.id());

  dwm1000Anchor.setup();

  // compass.setup();
  //  us.setup();
  //  drive.setup();

  sys.setup();
  led.setup();
  config.save();

  while (1) {
    eb.eventLoop();
    esp_task_wdt_reset();
    //    vTaskDelay(1);
  }
}

void measure(const char* s) {
  static uint64_t startTime;
  uint32_t delta;
  delta = Sys::millis() - startTime;
  startTime = Sys::millis();
  if (delta > 20) WARN(" slow %s :  %d msec", s, delta);
}
