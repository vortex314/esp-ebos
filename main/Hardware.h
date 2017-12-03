#ifndef HARDWARE_H
#define HARDWARE_H
#include <Bytes.h>
#include <Erc.h>

#include <driver/gpio.h>
#include <driver/i2c.h>

typedef void (*FunctionPointer)(void*);

class ByteStream {
 public:
  virtual Erc write(Bytes& bytes) = 0;
  virtual Erc write(uint8_t b) = 0;
  virtual uint8_t read() = 0;
  virtual void onRxd(FunctionPointer) = 0;
  virtual void onTxd(FunctionPointer) = 0;
  virtual uint32_t hasSpace() = 0;
  virtual uint32_t hasData() = 0;
};

class UART : public ByteStream {
 public:
  UART(uint32_t);
  Erc init();
  Erc deInit();

  Erc write(Bytes& bytes);
  Erc write(uint8_t b);
  Erc read(Bytes& bytes);
  uint8_t read();
  void onRxd(FunctionPointer);
  void onTxd(FunctionPointer);
  uint32_t hasSpace();
  uint32_t hasData();
};

//===================================================== GPIO DigitalIn ========

class DigitalIn {
 public:
  typedef enum { DIN_RAISE, DIN_FALL, DIN_CHANGE } PinChange;
  DigitalIn(uint32_t pin);
  int read();
  Erc init();
  Erc onChange(PinChange pinChange, FunctionPointer fp, void* object);

 private:
  uint32_t _gpio;
  FunctionPointer _fp;
  PinChange _pinChange;
  void* _object;

  /*static void gpio_isr_handler(void* arg) {
    if (_fp) _fp(arg);
  }*/
};
//===================================================== GPIO DigitalOut
class DigitalOut {
  uint32_t _gpio;

 public:
  DigitalOut(uint32_t pin);
  Erc init();
  Erc write(int);
};
//===================================================== I2C ===

#define I2C_WRITE_BIT
#define I2C_READ_BIT 0
class I2C {
  Bytes _txd;
  Bytes _rxd;

  i2c_port_t _port;
  uint32_t _scl;
  uint32_t _sda;
  uint32_t _clock;
  uint8_t _slaveAddress;

  FunctionPointer _onTxd;

 public:
  I2C(i2c_port_t idx, uint32_t scl, uint32_t sda);
  ~I2C();
  Erc init();
  Erc deInit();

  Erc setClock(uint32_t clock) {
    _clock = clock;
    return E_OK;
  }
  Erc setSlaveAddress(uint8_t slaveAddress) {
    _slaveAddress = slaveAddress;
    return E_OK;
  }

  Erc write(uint8_t* data, uint32_t size);
  Erc read(uint8_t* data, uint32_t size);
  Erc write(uint8_t data);
};

class SPI : public ByteStream {};

class ADC {};

class UEXT {
 public:
  UEXT(uint32_t idx);
  UART& getUART();
  SPI& getSPI();
  I2C& getI2C();
};

#endif