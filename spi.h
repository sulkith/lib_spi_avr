#ifndef TWI_H
#define TWI_H

#include <stdint.h>

#ifndef SPI_FREQ
#define SPI_FREQ 100000UL
#endif

#ifndef SPI_BUFFER_LENGTH
#define SPI_BUFFER_LENGTH 32
#endif

void spi_init();
void spi_write(uint8_t* data, uint8_t length, void (*callback)(uint8_t *));
void spi_read(uint8_t length, void (*callback)(uint8_t *));
uint8_t *spi_wait();

#endif
