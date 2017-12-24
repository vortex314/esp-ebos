#ifndef HARDWARE_H
#define HARDWARE_H
#include <Bytes.h>
#include <Erc.h>

class Bytes;

typedef void (*FunctionPointer)(void*);

typedef unsigned int uint32_t;
typedef uint32_t Erc;
typedef unsigned char uint8_t;
typedef uint32_t PhysicalPin;
typedef enum {
  LP_TXD = 0,
  LP_RXD,
  LP_SCL,
  LP_SDA,
  LP_MISO,
  LP_MOSI,
  LP_SCK,
  LP_CS
} LogicalPin;

class Driver {
 public:
  virtual Erc init() = 0;
  virtual Erc deInit() = 0;
};

class UART : public Driver {
  FunctionPointer _onRxd;
  FunctionPointer _onTxd;
  void* _onRxdVoid;
  void* _onTxdVoid;

 public:
  UART(uint32_t);
  UART(PhysicalPin txd, PhysicalPin rxd);
  Erc init();
  Erc deInit();

  Erc write(const uint8_t* data, uint32_t length);
  Erc write(uint8_t b);
  Erc read(Bytes& bytes);
  uint8_t read();
  void onRxd(FunctionPointer, void*);
  void onTxd(FunctionPointer, void*);
  uint32_t hasSpace();
  uint32_t hasData();
};

//===================================================== GPIO DigitalIn ========

class DigitalIn : public Driver {
 public:
  typedef enum { DIN_NONE, DIN_RAISE, DIN_FALL, DIN_CHANGE } PinChange;
  DigitalIn(PhysicalPin pin);
  int read();
  Erc init();
  Erc deInit();
  Erc onChange(PinChange pinChange, FunctionPointer fp, void* object);
  PhysicalPin getPin();

 private:
  PhysicalPin _gpio;
  FunctionPointer _fp;
  PinChange _pinChange;
  void* _object;

  /*static void gpio_isr_handler(void* arg) {
    if (_fp) _fp(arg);
  }*/
};
//===================================================== GPIO DigitalOut
class DigitalOut : public Driver {
  PhysicalPin _gpio;

 public:
  DigitalOut(PhysicalPin pin);
  Erc init();
  Erc deInit();
  Erc write(int);
  PhysicalPin getPin();
};
//===================================================== I2C ===

#define I2C_WRITE_BIT
#define I2C_READ_BIT 0

class I2C : public Driver {
 public:
  static I2C& create(PhysicalPin scl, PhysicalPin sda);
  ~I2C();
  virtual Erc init() = 0;
  virtual Erc deInit() = 0;
  virtual Erc setClock(uint32_t) = 0;
  virtual Erc setSlaveAddress(uint8_t address) = 0;
  virtual Erc write(uint8_t* data, uint32_t size) = 0;
  virtual Erc write(uint8_t data) = 0;
  virtual Erc read(uint8_t* data, uint32_t size) = 0;
};

class SPI : public Driver {
 public:
  typedef enum { SPI_MODE_0, SPI_MODE_1, SPI_MODE_2, SPI_MODE_3 } SPIMode;

  static SPI& create(PhysicalPin miso, PhysicalPin mosi, PhysicalPin sck,
                     PhysicalPin cs);
  ~SPI();
  virtual Erc init() = 0;
  virtual Erc deInit() = 0;
  virtual Erc exchange(Bytes& in, Bytes& out) = 0;
  virtual Erc onExchange(FunctionPointer, void*) = 0;
  virtual Erc setClock(uint32_t) = 0;
  virtual Erc setMode(SPIMode) = 0;
  virtual Erc setLsbFirst(bool) = 0;
};

class ADC {
  uint32_t _pin;

 public:
  ADC(uint32_t pin);
  Erc init();
  float getValue();
};

class Connector {
  uint32_t _pinsUsed;
  uint32_t _connectorIdx;
  uint32_t _physicalPins[8];
  UART* _uart;
  SPI* _spi;
  I2C* _i2c;

 private:
  uint32_t toPin(uint32_t logicalPin);
  void lockPin(LogicalPin);
  bool isUsedPin(LogicalPin lp) { return _pinsUsed & lp; }
  void freePin(LogicalPin);

 public:
  Connector(uint32_t idx);
  UART& getUART();
  SPI& getSPI();
  I2C& getI2C();
  DigitalIn& getDigitalIn(LogicalPin);
  DigitalOut& getDigitalOut(LogicalPin);
  ADC& getADC();
  // PWM& getPWM();
};

#endif