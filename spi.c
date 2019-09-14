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
	DDRB|=_BV(PB5)|_BV(PB3)|_BV(PB2);//set SCK MOSI CS as output
	SPCR = _BV(SPIE)|_BV(MSTR);
	SPSR = 0x01;//double the SPI speed.

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
	SPDR = 0xFF;
}

void spi_done() {
	uint8_t *data = &transmission.buffer[0];

	busy = SPI_IDLE;

	if (transmission.callback != NULL) {
		transmission.callback(data);
	}
}

void spi_write(uint8_t* data, uint8_t length, void (*callback)(uint8_t *)) {
	spi_wait();

	busy = SPI_TX;

	transmission.length = length;
	transmission.index = 1;
	transmission.callback = callback;
	memcpy(&transmission.buffer[0], data, length);

	spi_send(transmission.buffer[0]);
}

void spi_read(uint8_t length, void (*callback)(uint8_t *)) {
	spi_wait();

	busy = SPI_RX;

	transmission.length = length;
	transmission.index = 0;
	transmission.callback = callback;

	spi_send(0xFF);
}

ISR(SPI_STC_vect) {
	if(busy == SPI_RX)
	{
		spi_recv();
		if (transmission.index > transmission.length) {
			spi_done();
		}
	}
	else if(busy == SPI_TX)
	{
		if (transmission.index < transmission.length) {
			spi_send(transmission.buffer[transmission.index++]);
		} else {
			spi_done();
		}
	}
	else
	{
		spi_done();
	}
}
