#include <Hardware.h>

#include <Log.h>
#include <driver/gpio.h>
#include <driver/i2c.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static struct ESP32 { i2c_port_t _i2c_port; } esp32 = {I2C_NUM_0};

/**********************************************************************
 *
######                                                    ###
#     #     #     ####      #     #####    ##    #         #     #    #
#     #     #    #    #     #       #     #  #   #         #     ##   #
#     #     #    #          #       #    #    #  #         #     # #  #
#     #     #    #  ###     #       #    ######  #         #     #  # #
#     #     #    #    #     #       #    #    #  #         #     #   ##
######      #     ####      #       #    #    #  ######   ###    #    #

 *
 * *******************************************************************/
//================================================== DigitalIn =====
DigitalIn::DigitalIn(uint32_t pin) : _gpio(pin), _pinChange(DIN_NONE) {}

Erc DigitalIn::init() {
  esp_err_t erc;
  INFO(" DigitalIn Init %d ", _gpio);
  erc = gpio_set_direction((gpio_num_t)_gpio, GPIO_MODE_INPUT);
  if (erc) ERROR("gpio_set_direction():%d", erc);
  // interrupt of rising edge
  gpio_config_t io_conf;
  ZERO(io_conf);

  gpio_int_type_t interruptType = (gpio_int_type_t)GPIO_PIN_INTR_DISABLE;
  if (_pinChange == DIN_RAISE) {
    interruptType = (gpio_int_type_t)GPIO_PIN_INTR_POSEDGE;
  } else if (_pinChange == DIN_FALL) {
    interruptType = (gpio_int_type_t)GPIO_PIN_INTR_NEGEDGE;
  } else if (_pinChange == DIN_CHANGE) {
    interruptType = (gpio_int_type_t)GPIO_INTR_ANYEDGE;
  }
  io_conf.intr_type = interruptType;

  io_conf.pin_bit_mask = (1 << _gpio);    // bit mask of the pins
  io_conf.mode = GPIO_MODE_INPUT;         // set as input mode
  io_conf.pull_up_en = (gpio_pullup_t)0;  // enable pull-up mode

#define ESP_INTR_FLAG_DEFAULT 0
  if (_fp) {
    // install gpio isr service
    erc = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (erc) ERROR("gpio_install_isr_service() :%d", erc);
    // hook isr handler for specific gpio pin
    if (_fp) erc = gpio_isr_handler_add((gpio_num_t)_gpio, _fp, (void*)_object);
    if (erc) ERROR("gpio_isr_handler_add() :%d", erc);
  }
  return gpio_config(&io_conf);
}

Erc DigitalIn::deInit() { return E_OK; }

Erc DigitalIn::onChange(PinChange pinChange, FunctionPointer fp, void* object) {
  _pinChange = pinChange;
  _fp = fp;
  _object = object;
  return E_OK;
}

int DigitalIn::read() { return gpio_get_level((gpio_num_t)_gpio); }

PhysicalPin DigitalIn::getPin() { return _gpio; }
/*
######                                                  #######
#     #     #     ####      #     #####    ##    #      #     #  #    #   #####
#     #     #    #    #     #       #     #  #   #      #     #  #    #     #
#     #     #    #          #       #    #    #  #      #     #  #    #     #
#     #     #    #  ###     #       #    ######  #      #     #  #    #     #
#     #     #    #    #     #       #    #    #  #      #     #  #    #     #
######      #     ####      #       #    #    #  ###### #######   ####      #
*/
//================================================== DigitalOu =====
DigitalOut::DigitalOut(uint32_t pin) : _gpio(pin) {}

Erc DigitalOut::init() {
  INFO(" DigitalOut Init %d ", _gpio);
  esp_err_t erc = gpio_set_direction((gpio_num_t)_gpio, GPIO_MODE_OUTPUT);
  if (erc) ERROR("gpio_set_direction():%d", erc);
  gpio_config_t io_conf;
  // disable interrupt
  io_conf.intr_type = (gpio_int_type_t)GPIO_PIN_INTR_DISABLE;
  // set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  // bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pin_bit_mask = 1 << _gpio;
  // disable pull-down mode
  io_conf.pull_down_en = (gpio_pulldown_t)0;
  // disable pull-up mode
  io_conf.pull_up_en = (gpio_pullup_t)0;
  // configure GPIO with the given settings
  return gpio_config(&io_conf);
}

Erc DigitalOut::deInit() { return E_OK; }

Erc DigitalOut::write(int x) {
  if (x) {
    return gpio_set_level((gpio_num_t)_gpio, 1);
  } else {
    return gpio_set_level((gpio_num_t)_gpio, 0);
  }
}

PhysicalPin DigitalOut::getPin() { return _gpio; }
/**********************************************************************
 *
                                *###    #####   #####
                                  #    #     # #     #
                                  #          # #
                                  #     #####  #
                                  #    #       #
                                  #    #       #     #
                                 ###   #######  #####

 *
 * *******************************************************************/
//================================================== I2C =======
/*
 * * */
//#define READ_BIT I2C_MASTER_READ /*!< I2C master read */
#define ACK_CHECK_EN 0x1  /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0       /*!< I2C ack value */
#define NACK_VAL 0x1      /*!< I2C nack value */

class I2C_ESP32 : public I2C {
  Bytes _txd;
  Bytes _rxd;

  PhysicalPin _scl;
  PhysicalPin _sda;
  uint32_t _clock;
  uint8_t _slaveAddress;

  FunctionPointer _onTxd;

  i2c_port_t _port;

 public:
  I2C_ESP32(PhysicalPin scl, PhysicalPin sda);
  ~I2C_ESP32();
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

I2C_ESP32::I2C_ESP32(PhysicalPin scl, PhysicalPin sda)
    : _txd(16), _rxd(16), _scl(scl), _sda(sda) {
  _port = esp32._i2c_port;
  _clock = 100000;
  _slaveAddress = 0x1E;  // HMC 5883L
}

I2C_ESP32::~I2C_ESP32() {
  esp_err_t erc = i2c_driver_delete(_port);
  INFO(" erc : %d ", erc);
}

Erc I2C_ESP32::init() {
  INFO(" init : scl:%d ,sda :%d ", _scl, _sda);
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = (gpio_num_t)_sda;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = (gpio_num_t)_scl;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = _clock;
  esp_err_t erc = i2c_param_config(_port, &conf);
  INFO("i2c_param_config() : %d", erc);
  erc = i2c_driver_install(_port, conf.mode, 0, 0,
                           0);  // buffer sizes 0 formaster
  INFO("i2c_driver_install() : %d", erc);
  return erc;
}

Erc I2C_ESP32::deInit() { return E_OK; }

Erc I2C_ESP32::write(uint8_t* data, uint32_t size) {
  esp_err_t erc;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  erc = i2c_master_start(cmd);
  if (erc) INFO("i2c_master_start(cmd):%d", erc)
  erc = i2c_master_write_byte(cmd, (_slaveAddress << 1) | I2C_MASTER_WRITE,
                              ACK_CHECK_EN);
  if (erc) INFO("i2c_master_write_byte():%d", erc)
  erc = i2c_master_write(cmd, data, size, ACK_CHECK_EN);
  if (erc) INFO("i2c_master_write():%d", erc);
  erc = i2c_master_stop(cmd);
  if (erc) INFO("i2c_master_stop():%d", erc);
  erc = i2c_master_cmd_begin(_port, cmd, 1000 / portTICK_RATE_MS);
  if (erc) INFO("i2c_master_cmd_begin():%d", erc);
  i2c_cmd_link_delete(cmd);
  return erc;
}

Erc I2C_ESP32::write(uint8_t b) { return write(&b, 1); }

Erc I2C_ESP32::read(uint8_t* data, uint32_t size) {
  if (size == 0) {
    return E_OK;
  }
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (_slaveAddress << 1) | I2C_MASTER_READ,
                        ACK_CHECK_EN);
  if (size > 1) {
    i2c_master_read(cmd, data, size - 1, ACK_VAL);
  }
  i2c_master_read_byte(cmd, data + size - 1, NACK_VAL);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(_port, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

I2C& I2C::create(PhysicalPin scl, PhysicalPin sda) {
  I2C_ESP32* ptr = new I2C_ESP32(scl, sda);
  return *ptr;
}

//========================================================   A D C
#include "esp_adc_cal.h"

esp_adc_cal_characteristics_t _characteristics;

#define V_REF 1100
//#define ADC1_TEST_CHANNEL (ADC1_CHANNEL_6)  // GPIO 34
#define ADC1_TEST_CHANNEL (ADC1_CHANNEL_5)  // GPIO 33

ADC::ADC(uint32_t pin) { _pin = pin; }

Erc ADC::init() {
  esp_err_t erc;
  erc = adc1_config_width(ADC_WIDTH_BIT_12);
  if (erc) ERROR("adc1_config_width(): %d", erc);
  erc = adc1_config_channel_atten(ADC1_TEST_CHANNEL, ADC_ATTEN_DB_11);
  if (erc) ERROR("adc1_config_channel_atten():%d", erc);
  esp_adc_cal_get_characteristics(V_REF, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12,
                                  &_characteristics);

  return E_OK;
}

float ADC::getValue() {
  uint32_t voltage = adc1_to_voltage(ADC1_TEST_CHANNEL, &_characteristics);
  return voltage / 1000.0;
}

  /*******************************************************************************

                                           #####  ######    ###
                                          #     # #     #    #
                                          #       #     #    #
                                           #####  ######     #
                                                # #          #
                                          #     # #          #
                                           #####  #         ###

  *****************************************************************************/

#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_system.h"

class SPI_ESP32 : public SPI {
 protected:
  FunctionPointer _onExchange;
  uint32_t _clock;
  uint32_t _mode;
  bool _lsbFirst;
  PhysicalPin _miso, _mosi, _sck, _cs;
  void* _object;
  spi_device_handle_t _spi;

 public:
  SPI_ESP32(PhysicalPin miso, PhysicalPin mosi, PhysicalPin sck, PhysicalPin cs)
      : _miso(miso), _mosi(mosi), _sck(sck), _cs(cs) {
    _clock = 100000;
    _mode = 0;
  };

  Erc init() {
    DEBUG(" SPI_ESP32 : miso : %d, mosi : %d , sck : %d , cs : %d ", _miso,
          _mosi, _sck, _cs);

    esp_err_t ret;

    spi_bus_config_t buscfg;
    buscfg.miso_io_num = _miso;
    buscfg.mosi_io_num = _mosi;
    buscfg.sclk_io_num = _sck;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 0;

    // Initialize the SPI bus
    ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    if (ret) {
      ERROR("spi_bus_initialize(HSPI_HOST, &buscfg, 1) = %d ", ret);
      return EIO;
    }

    spi_device_interface_config_t devcfg;
    memset(&devcfg, 0, sizeof(devcfg));
    devcfg.clock_speed_hz = _clock;  // Clock out at 10 MHz
    devcfg.mode = _mode;             // SPI mode 0
    devcfg.spics_io_num = _cs;       // CS pin
    devcfg.queue_size = 7;
    // We want to be able to queue 7 transactions at a time
    devcfg.pre_cb = 0;
    // Specify pre-transfer callback to handle D/C line
    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &_spi);
    if (ret) {
      ERROR("spi_bus_add_device(HSPI_HOST, &devcfg, &_spi) = %d ", ret);
      return EIO;
    }
    return E_OK;
  };

  Erc deInit() {
    esp_err_t ret = spi_bus_remove_device(_spi);
    if (ret) {
      ERROR("spi_bus_remove_device(_spi) = %d ", ret);
      return EIO;
    }
    ret = spi_bus_free(HSPI_HOST);
    if (ret) {
      ERROR("spi_bus_free(HSPI_HOST) = %d ", ret);
      return EIO;
    }
    return E_OK;
  }

  Erc exchange(Bytes& in, Bytes& out) {
    uint8_t inData[100];
    esp_err_t ret;
    spi_transaction_t t, *pTrans;
    if (out.length() == 0) return E_INVAL;  // no need to send anything
    memset(&t, 0, sizeof(t));               // Zero out the transaction
    t.length = out.length() * 8;
    // Len is in bytes, transaction length is in bits.
    t.tx_buffer = out.data();  // Data
    t.rx_buffer = inData;
    //    t.flags = SPI_TRANS_USE_RXDATA;
    t.user = (void*)1;  // D/C needs to be set to 1
    ret = spi_device_queue_trans(_spi, &t, 1000);
    if (ret) {
      ERROR("spi_device_queue_trans(_spi, &t, 1000) = %d ", ret);
      return EIO;
    }
    ret = spi_device_get_trans_result(_spi, &pTrans, 1000);
    if (ret) {
      ERROR("spi_device_get_trans_result(_spi, &pTrans, 1000) = %d ", ret);
      return EIO;
    }
    in.clear();
    for (int i = 0; i < out.length(); i++) {
      in.write(inData[i]);
    }
    return E_OK;  // Should have had no issues.
  };

  Erc setClock(uint32_t clock) {
    _clock = clock;
    return E_OK;
  }

  Erc setMode(SPIMode mode) {
    _mode = mode;
    return E_OK;
  }

  Erc setLsbFirst(bool f) {
    _lsbFirst = f;
    return true;
  }

  Erc onExchange(FunctionPointer p, void* ptr) { return E_OK; }
};

SPI::~SPI() {}

SPI& SPI::create(PhysicalPin miso, PhysicalPin mosi, PhysicalPin sck,
                 PhysicalPin cs) {
  SPI_ESP32* ptr = new SPI_ESP32(miso, mosi, sck, cs);
  return *ptr;
}

/*
 #####
#     #   ####   #    #  #    #  ######   ####    #####   ####   #####
#        #    #  ##   #  ##   #  #       #    #     #    #    #  #    #
#        #    #  # #  #  # #  #  #####   #          #    #    #  #    #
#        #    #  #  # #  #  # #  #       #          #    #    #  #####
#     #  #    #  #   ##  #   ##  #       #    #     #    #    #  #   #
 #####    ####   #    #  #    #  ######   ####      #     ####   #    #

*/
Connector::Connector(uint32_t idx) {  // defined by PCB layout
  if (idx == 1) {
    _physicalPins[LP_TXD] = 17;
    _physicalPins[LP_RXD] = 16;
    _physicalPins[LP_SCL] = 15;
    _physicalPins[LP_SDA] = 4;
    _physicalPins[LP_MISO] = 19;
    _physicalPins[LP_MOSI] = 23;
    _physicalPins[LP_SCK] = 18;
    _physicalPins[LP_CS] = 21;
  } else if (idx == 2) {
    _physicalPins[LP_TXD] = 32;
    _physicalPins[LP_RXD] = 33;
    _physicalPins[LP_SCL] = 25;
    _physicalPins[LP_SDA] = 26;
    _physicalPins[LP_MISO] = 27;
    _physicalPins[LP_MOSI] = 14;
    _physicalPins[LP_SCK] = 13;
    _physicalPins[LP_CS] = 12;
  } else if (idx == 3) {
    _physicalPins[LP_TXD] = 1;
    _physicalPins[LP_RXD] = 3;
    _physicalPins[LP_SCL] = 22;
    _physicalPins[LP_SDA] = 5;
    _physicalPins[LP_MISO] = 36;
    _physicalPins[LP_MOSI] = 39;
    _physicalPins[LP_SCK] = 34;
    _physicalPins[LP_CS] = 35;
  }
  _spi = 0;
  _i2c = 0;
  _uart = 0;
  _pinsUsed = 0;
  _connectorIdx = idx;
}

PhysicalPin Connector::toPin(uint32_t logicalPin) {
  INFO(" logical %d => %d physial ", logicalPin, _physicalPins[logicalPin]);
  return _physicalPins[logicalPin];
}

UART& Connector::getUART() {
  lockPin(LP_TXD);
  lockPin(LP_RXD);
  _uart = new UART(toPin(LP_TXD), toPin(LP_RXD));
  return *_uart;
};

SPI& Connector::getSPI() {
  _spi = new SPI_ESP32(toPin(LP_MISO), toPin(LP_MOSI), toPin(LP_SCK),
                       toPin(LP_CS));
  lockPin(LP_MISO);
  lockPin(LP_MOSI);
  lockPin(LP_SCK);
  lockPin(LP_CS);
  return *_spi;
}
I2C& Connector::getI2C() {
  lockPin(LP_SDA);
  lockPin(LP_SCL);
  _i2c = new I2C_ESP32(toPin(LP_SCL), toPin(LP_SDA));
  return *_i2c;
};

DigitalOut& Connector::getDigitalOut(LogicalPin lp) {
  lockPin(lp);
  DigitalOut* _out = new DigitalOut(toPin(lp));
  return *_out;
}

DigitalIn& Connector::getDigitalIn(LogicalPin lp) {
  lockPin(lp);
  DigitalIn* _in = new DigitalIn(toPin(lp));
  return *_in;
}

void Connector::lockPin(LogicalPin lp) {
  if (_pinsUsed & lp) ERROR(" PIN in use %d >>>>>>>>>>>>>>>>>> ", lp);
  _pinsUsed |= (1 << lp);
}

/*


@     @    @    @@@@@@  @@@@@@@
@     @   @ @   @     @    @
@     @  @   @  @     @    @
@     @ @     @ @@@@@@     @
@     @ @@@@@@@ @   @      @
@     @ @     @ @    @     @
 @@@@@  @     @ @     @    @


  @@@    @@@@@   @@@@@
   @    @     @ @     @
   @          @ @
   @     @@@@@   @@@@@
   @    @             @
   @    @       @     @
  @@@   @@@@@@@  @@@@@

   @    @@@@@@   @@@@@
  @ @   @     @ @     @
 @   @  @     @ @
@     @ @     @ @
@@@@@@@ @     @ @
@     @ @     @ @     @
@     @ @@@@@@   @@@@@



*/
/*
Uart& getUart() {
    lockPin(LP_TXD);
    lockPin(LP_RXD);
    return Uart(LP_TXD, LP_RXD);
  };
  void freeUart();
  Spi& getSpi() { return Spi(LP_MISO, LP_MOSI, LP_SCK, LP_CS); };
  I2C& getI2C() {
    lockPin(LP_SDA);
    lockPin(LP_SCL);
    return I2C(LP_SDA, LP_SCL);
  };*/