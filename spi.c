#include <avr/io.h>
#include <avr/interrupt.h>
//#include <util/twi.h>
#include <string.h>

#include "spi.h"

#define SPI_RX 1
#define SPI_TX 2
#define SPI_IDLE 0
static volatile uint8_t busy;
static struct {
  uint8_t buffer[SPI_BUFFER_LENGTH];
  uint8_t length;
  uint8_t index;
  void (*callback)(uint8_t *);
} transmission;

void spi_init() {
  SPCR = _BV(SPIE)|_BV(MSTR);

  busy = SPI_IDLE;

  sei();

  SPCR |= _BV(SPE);
}

uint8_t *spi_wait() {
  while (busy);
  return &transmission.buffer[0];
}

void spi_start(void) {
  SPCR |= _BV(SPE);
}

void spi_stop(void) {
  SPCR &= ~_BV(SPE);
}

void spi_send(uint8_t data) {
  SPDR = data;
}

void spi_recv() {
  transmission.buffer[transmission.index++] = SPDR;
}

void spi_done() {
//  uint8_t address = transmission.buffer[0] >> 1; no address for SPI
  uint8_t *data = &transmission.buffer[0];

  busy = SPI_IDLE;

  if (transmission.callback != NULL) {
    transmission.callback(data);
  }
}

void spi_write(uint8_t* data, uint8_t length, void (*callback)(uint8_t *)) {
  spi_wait();

  busy = SPI_TX;

//  transmission.buffer[0] = (address << 1) | TW_WRITE;no address for SPI
  transmission.length = length;
  transmission.index = 0;
  transmission.callback = callback;
  memcpy(&transmission.buffer[0], data, length);

  spi_start();
}

void spi_read(uint8_t length, void (*callback)(uint8_t *)) {
  spi_wait();

  busy = SPI_RX;

  transmission.length = length;
  transmission.index = 0;
  transmission.callback = callback;

  spi_start();
}

ISR(SPI_STC_vect) {
if(busy == SPI_RX)
{
    spi_recv();
}
else if(busy == SPI_TX)
{
    if (transmission.index < transmission.length) {
      spi_send(transmission.buffer[transmission.index++]);
    } else {
	spi_stop();
	spi_done();
    }
}
else
{
	spi_stop();
	spi_done();
}

}
