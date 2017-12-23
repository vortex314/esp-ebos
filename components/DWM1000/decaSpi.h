
#ifndef DECA_SPI_H
#define DECA_SPI_H
#include <Hardware.h>
void spi_set_global(SPI* spi);
extern "C" {
void spi_set_rate_low();
void spi_set_rate_high();
}
#endif
