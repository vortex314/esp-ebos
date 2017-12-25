#include <Hardware.h>
#include <Log.h>
#include <deca_device_api.h>
#include <esp_attr.h>

SPI *_gSpi;  // to support irq

void spi_set_global(SPI *spi) {
  INFO(" setting global SPI");
  _gSpi = spi;
}

void spi_cs_select() {}

void spi_cs_deselect() {}

Bytes out(100);
Bytes in(100);
Str hex(200);

//////////////////////////////////////////////////////////////////////////////////
//
//
//
/////////////////////////////////////////////////////////////////////////////////
extern "C" int IRAM_ATTR writetospi(uint16 hLen, const uint8 *hbuff,
                                    uint32 bLen, const uint8 *buffer) {
  //       INFO(" %X write to SPI %d %d ",_gSpi,hLen,bLen);
  out.clear();
  in.clear();
  out.write((uint8_t *)hbuff, 0, hLen);
  out.write((uint8_t *)buffer, 0, bLen);

  _gSpi->exchange(in, out);

  return 0;
}
//////////////////////////////////////////////////////////////////////////////////
//
//
//
/////////////////////////////////////////////////////////////////////////////////

extern "C" int IRAM_ATTR readfromspi(uint16 hLen, const uint8 *hbuff,
                                     uint32 bLen, uint8 *buffer) {
  //          INFO(" %X read from SPI %d %d ",_gSpi,hLen,bLen);
  out.clear();
  in.clear();
  out.write((uint8_t *)hbuff, 0, hLen);
  out.write((uint8_t *)buffer, 0, bLen);

  _gSpi->exchange(in, out);
  /*
  hex.clear();
  INFO(" in[%d] : %s ",in.length(),in.toHex(hex));
  hex.clear();
  INFO(" out[%d] : %s ",out.length(),out.toHex(hex));*/
  in.offset(hLen);
  int count = 0;
  while (in.hasData() && (count < bLen)) {
    *buffer++ = in.read();
    count++;
  }
  return 0;
}
//////////////////////////////////////////////////////////////////////////////////
//
//
//
/////////////////////////////////////////////////////////////////////////////////
extern "C" void spi_set_rate_low() {
  INFO(" spi_set_rate_low() %X", _gSpi);
  _gSpi->deInit();
  _gSpi->setClock(1000000);
  _gSpi->init();
}
//////////////////////////////////////////////////////////////////////////////////
//
//
//
/////////////////////////////////////////////////////////////////////////////////
extern "C" void spi_set_rate_high() {
  INFO(" spi_set_rate_high() %X", _gSpi);
  _gSpi->deInit();
  _gSpi->setClock(8000000);
  _gSpi->init();
}

extern "C" decaIrqStatus_t decamutexon() { return 0; }

extern "C" void decamutexoff(decaIrqStatus_t s) {}
