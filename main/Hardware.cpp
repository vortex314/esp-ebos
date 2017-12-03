#include <Hardware.h>

#include <Log.h>
#include <driver/i2c.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//================================================== DigitalIn =====
DigitalIn::DigitalIn(uint32_t pin) : _gpio(pin) {}

Erc DigitalIn::init() {
  esp_err_t erc;
  INFO(" DigitaLin Init %d ", _gpio);
  erc = gpio_set_direction((gpio_num_t)_gpio, GPIO_MODE_INPUT);
  if (erc) ERROR("gpio_set_direction():%d", erc);
  // interrupt of rising edge
  gpio_config_t io_conf;
  gpio_int_type_t interruptType = (gpio_int_type_t)GPIO_PIN_INTR_POSEDGE;

  if (_pinChange == DIN_RAISE) {
    interruptType = (gpio_int_type_t)GPIO_PIN_INTR_POSEDGE;
  } else if (_pinChange == DIN_FALL) {
    interruptType = (gpio_int_type_t)GPIO_PIN_INTR_NEGEDGE;
  }
  io_conf.intr_type = interruptType;
  // bit mask of the pins, use GPIO4/5 here
  io_conf.pin_bit_mask = (1 << _gpio);
  // set as input mode
  io_conf.mode = GPIO_MODE_INPUT;
  // enable pull-up mode
  io_conf.pull_up_en = (gpio_pullup_t)0;

#define ESP_INTR_FLAG_DEFAULT 0
  // install gpio isr service
  erc = gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  if (erc) ERROR("gpio_install_isr_service() :%d", erc);
  // hook isr handler for specific gpio pin
  if (_fp) erc = gpio_isr_handler_add((gpio_num_t)_gpio, _fp, (void*)_object);
  if (erc) ERROR("gpio_isr_handler_add() :%d", erc);
  return gpio_config(&io_conf);
}

Erc DigitalIn::onChange(PinChange pinChange, FunctionPointer fp, void* object) {
  _pinChange = pinChange;
  _fp = fp;
  _object = object;
  return E_OK;
}

//================================================== DigitalOu =====
DigitalOut::DigitalOut(uint32_t pin) : _gpio(pin) {}

Erc DigitalOut::init() {
  INFO(" DigitaLin Init %d ", _gpio);
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

Erc DigitalOut::write(int x) {
  if (x) {
    return gpio_set_level((gpio_num_t)_gpio, 1);
  } else {
    return gpio_set_level((gpio_num_t)_gpio, 0);
  }
}

//================================================== I2C =======
/*
 * * */
//#define READ_BIT I2C_MASTER_READ /*!< I2C master read */
#define ACK_CHECK_EN 0x1  /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0       /*!< I2C ack value */
#define NACK_VAL 0x1      /*!< I2C nack value */

I2C::I2C(i2c_port_t idx, uint32_t scl, uint32_t sda)
    : _txd(16), _rxd(16), _scl(scl), _sda(sda) {
  _port = idx;
  _clock = 100000;
  _slaveAddress = 0x1E;  // HMC 5883L
}

I2C::~I2C() {
  esp_err_t erc = i2c_driver_delete(_port);
  INFO(" erc : %d ", erc);
}

Erc I2C::init() {
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

Erc I2C::write(uint8_t* data, uint32_t size) {
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

Erc I2C::write(uint8_t b) { return write(&b, 1); }

Erc I2C::read(uint8_t* data, uint32_t size) {
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

/*
void loadHardware(){
        uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                .rx_flow_ctrl_thresh =1
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_2, GPIO_NUM_17, GPIO_NUM_16, UART_PIN_NO_CHANGE,
UART_PIN_NO_CHANGE); uart_driver_install(UART_NUM_2, BUF_SIZE * 2, 0, 0, NULL,
0);

        // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    while (1) {
        // Read data from the UART
        int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 20 /
portTICK_RATE_MS);
        // Write data back to the UART
        uart_write_bytes(UART_NUM_1, (const char *) data, len);
    }

}

*/